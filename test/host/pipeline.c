#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "output_lcd/render_lcd.h"
#include "output_lcd/lcd_driver.h"
#include "epd_metrics.h"
#include "output_common/render_method.h"
const enum EpdRenderMethod EPD_CURRENT_RENDER_METHOD=RENDER_METHOD_LCD;
static _Thread_local bool in_isr;
static _Thread_local int core;
static atomic_bool quit, abort_scan;
static atomic_int feeder_active, overlap, scans;
struct HostTask { pthread_t thread; SemaphoreHandle_t notify; RenderContext_t* ctx; int id; };
int64_t esp_timer_get_time(void) { struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000LL+t.tv_nsec/1000; }
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* s) {
 pthread_mutex_init(&s->lock,0);pthread_cond_init(&s->changed,0);s->count=s->max=1;return s;
}
SemaphoreHandle_t xSemaphoreCreateCounting(unsigned max,unsigned initial) { SemaphoreHandle_t s=calloc(1,sizeof(*s));xSemaphoreCreateMutexStatic(s);s->count=initial;s->max=max;return s; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return xSemaphoreCreateCounting(1,0); }
int xSemaphoreTake(SemaphoreHandle_t s,unsigned ms) {
 pthread_mutex_lock(&s->lock);int ok=1;
 struct timespec t;clock_gettime(CLOCK_REALTIME,&t);
 if(ms!=portMAX_DELAY && ms>300) ms=300; // Bound injected timeout cases.
 t.tv_sec+=ms/1000;t.tv_nsec+=(ms%1000)*1000000L;if(t.tv_nsec>=1000000000){++t.tv_sec;t.tv_nsec-=1000000000;}
 while(!s->count) {
  if(!ms) {ok=0;break;}
  int r=ms==portMAX_DELAY ? pthread_cond_wait(&s->changed,&s->lock) : pthread_cond_timedwait(&s->changed,&s->lock,&t);
  if(r){ok=0;break;}
 }
 if(ok)--s->count;pthread_mutex_unlock(&s->lock);return ok;
}
void xSemaphoreGive(SemaphoreHandle_t s) { pthread_mutex_lock(&s->lock);if(s->count<s->max)++s->count;pthread_cond_signal(&s->changed);pthread_mutex_unlock(&s->lock); }
void xSemaphoreGiveFromISR(SemaphoreHandle_t s,BaseType_t* w) { assert(in_isr);xSemaphoreGive(s);*w=1; }
void vSemaphoreDelete(SemaphoreHandle_t s){pthread_mutex_destroy(&s->lock);pthread_cond_destroy(&s->changed);free(s);}
void vTaskDelay(unsigned ms){if(ms)usleep(ms*1000);else sched_yield();}
int xPortGetCoreID(void){return core;} int xPortInIsrContext(void){return in_isr;}
void vTaskSuspendAll(void){} void xTaskResumeAll(void){}
void xTaskNotifyGive(TaskHandle_t t){xSemaphoreGive(t->notify);}
void epd_set_mode(bool mode){(void)mode;}
static pthread_t scanner;static bool scan_joinable;
static line_cb_func_t source;static frame_done_func_t done;static void *source_ctx,*done_ctx;
static int test_fault;static int64_t scan_at,last_scan;
static bool clearing;static unsigned clear_drive;
static bool delay_feeder;
static _Thread_local unsigned prepared_rows;
static lut_func_t real_lookup;
static void delayed_lookup(const uint32_t* data,uint8_t* output,const uint8_t* lut,uint32_t width) {
 if(delay_feeder && ++prepared_rows==150)usleep(110000);
 real_lookup(data,output,lut,width);
}
void epd_lcd_line_source_cb(line_cb_func_t cb,void* p){source=cb;source_ctx=p;}
void epd_lcd_frame_done_cb(frame_done_func_t cb,void* p){done=cb;done_ctx=p;}
void epd_lcd_get_stream_stats(int* a,int* b){*a=*b=0;}
int epd_lcd_get_scan_time_us(void){return last_scan;}
int64_t epd_lcd_get_spv_start_us(void){return scan_at;}
uint32_t epd_lcd_get_pixel_clock_hz(void){return 20000000;}
static int irq_core=1;
int epd_lcd_get_irq_core(void){return irq_core;}
static uint8_t phases[5*64];
static void* scan_run(void* p) {
 RenderContext_t* ctx=p;in_isr=true;scan_at=esp_timer_get_time();
 unsigned phase=ctx->scan_frame;uint8_t row[16];
 for(int y=0;y<ctx->lines_total && !atomic_load(&abort_scan);++y){
  if(test_fault==1 && y==16) break; // Incomplete LCD scan.
  if(test_fault==2 && y==16){while(!atomic_load(&abort_scan))usleep(100);break;} // Lost completion.
  source(source_ctx,row);
  if(clearing) {
   for(unsigned x=0;x<sizeof(row);++x) {
    unsigned expected=0;
    for(unsigned j=0;j<4;++j) {
     int pixel=x*4+j;
     if(y>=ctx->area.y && y<ctx->area.y+ctx->area.height &&
        pixel>=ctx->area.x && pixel<ctx->area.x+ctx->area.width)
      expected|=clear_drive<<(6-2*j);
    }
    assert(row[x]==expected);
   }
  } else {
  for(unsigned x=0;x<sizeof(row);++x){
   unsigned expected=0;
   for(unsigned j=0;j<4;++j){unsigned v=ctx->data_ptr[y*64+x*4+j];
    unsigned old=v&15,target=v>>4;
    expected|=((phases[(phase*16+target)*4+old/4]>>(6-2*(old%4)))&3)<<(6-2*j);
   }
   if(!test_fault)assert(row[x]==expected);
  }
  }
  usleep(50);
 }
 last_scan=esp_timer_get_time()-scan_at;
 if(!atomic_load(&abort_scan) && done)done(done_ctx);
 return 0;
}
void epd_lcd_start_frame(void){
 if(scan_joinable){pthread_join(scanner,0);scan_joinable=false;}
 atomic_store(&abort_scan,false);++scans;
#if EPD_LCD_FULL_PHASE
 if(!clearing){RenderContext_t* ctx=source_ctx;for(int y=0;y<ctx->lines_total;++y)
  assert(__atomic_load_n(&ctx->line_ready[y],__ATOMIC_ACQUIRE)>=ctx->scan_frame+1);}
#endif
 assert(pthread_create(&scanner,0,scan_run,source_ctx)==0);scan_joinable=true;
}
void epd_lcd_abort_frame(void){atomic_store(&abort_scan,true);if(scan_joinable){pthread_join(scanner,0);scan_joinable=false;}}
static void* feed_run(void* p){
 struct HostTask* t=p;core=t->id;
 while(xSemaphoreTake(t->notify,portMAX_DELAY)){
  if(atomic_load(&quit))break;
  ++feeder_active;
  if(t->ctx->current_frame>t->ctx->scan_frame)++overlap;
  prepared_rows=0;lcd_calculate_frame(t->ctx,t->id);
  --feeder_active;
  xSemaphoreGive(t->ctx->feed_done_smphr[t->id]);
 }
 return 0;
}
uint32_t epd_interlace_4bpp_line_VE(const uint8_t*,const uint8_t*,uint8_t*,uint8_t*,int);
int main(void){
 // Independent scalar checks include odd widths, tails and unaligned buffers.
 for (int width=1;width<=1216;++width) {
  uint8_t to[610],from[610],diff[1218],dirty[610],expected[1218],columns[610];
  for (int i=0;i<610;++i){to[i]=(i*71+width)&255;from[i]=(i*19+width/3)&255;dirty[i]=columns[i]=(i*7)&255;}
  uint32_t changed=0;
  for(int x=0;x<width;++x){unsigned t=(to[1+x/2]>>(4*(x%2)))&15,f=(from[1+x/2]>>(4*(x%2)))&15;expected[x]=(t<<4)|f;columns[1+x/2]|=(t^f)<<(4*(x%2));changed|=t^f;}
  uint32_t got=epd_interlace_4bpp_line_VE(to+1,from+1,diff+1,dirty+1,width);
  assert((got!=0)==(changed!=0));assert(!memcmp(diff+1,expected,width));assert(!memcmp(dirty,columns,610));
 }

 RenderContext_t ctx={0};ctx.display_width=64;ctx.display_height=ctx.lines_total=684;
 ctx.area=ctx.crop_to=(EpdRect){0,0,64,684};ctx.mode=MODE_GL16|MODE_PACKING_1PPB_DIFFERENCE;
 ctx.cycle_frames=5;ctx.conversion_lut=calloc(1,1024);
 uint8_t* data=malloc(64*684);for(int i=0;i<64*684;++i)data[i]=(i*71+i/7)&255;ctx.data_ptr=data;
 for(unsigned i=0;i<sizeof(phases);++i)phases[i]=(i*137+i/11)&255;
 EpdWaveformPhases ph={5,phases,0};const EpdWaveformPhases* range[]={&ph};
 EpdWaveformMode mode={MODE_GL16,1,range};const EpdWaveformMode* modes[]={&mode};
 EpdWaveform wf={1,1,modes,0};ctx.waveform=&wf;
 LutFunctionPair pair=find_lut_functions(ctx.mode,1024);ctx.lut_lookup_func=pair.lookup_func;ctx.lut_build_func=pair.build_func;
 assert(pair.lookup_func && pair.build_func);real_lookup=pair.lookup_func;ctx.lut_lookup_func=delayed_lookup;
 ctx.line_ready=calloc(684,1);ctx.line_threads=calloc(684,1);ctx.line_mask=calloc(16,1);
 ctx.prefill_done=xSemaphoreCreateCounting(2,0);ctx.frame_done=xSemaphoreCreateBinary();
 struct HostTask tasks[2];
 for(int i=0;i<2;++i){ctx.line_queues[i]=lq_init(EPD_LCD_FULL_PHASE ? epd_lcd_phase_queue_capacity(ctx.lines_total):128,16);ctx.feed_done_smphr[i]=xSemaphoreCreateBinary();ctx.feed_space[i]=xSemaphoreCreateBinary();tasks[i]=(struct HostTask){.notify=xSemaphoreCreateBinary(),.ctx=&ctx,.id=i};ctx.feed_tasks[i]=&tasks[i];pthread_create(&tasks[i].thread,0,feed_run,&tasks[i]);}
 // Repeat after both failure types: no old phase, callback, queue, or semaphore can leak.
 int scenarios[]={0,1,0,2,0,0};
 for(unsigned t=0;t<sizeof(scenarios)/sizeof(*scenarios);++t){
  test_fault=scenarios[t];irq_core=t%2;ctx.error=0;ctx.current_frame=0;epd_metrics_reset();
  delay_feeder=EPD_LCD_FULL_PHASE && t==5;
  lcd_do_update(&ctx);epd_lcd_abort_frame();
  EpdMetrics m=epd_metrics_get();assert(!atomic_load(&feeder_active));
  assert((ctx.error!=0)==(test_fault!=0));assert(m.faults==(test_fault!=0));
  if(!test_fault){assert(m.completed==5);assert(m.pixel_clock_hz==20000000);assert(!m.fault_phase && !m.fault_line && !m.fault_thread);}
  for(int i=0;i<2;++i){assert(atomic_load(&ctx.line_queues[i].current)==atomic_load(&ctx.line_queues[i].last));assert(!atomic_load(&ctx.feed_waiting[i]));}
 }
 // The clear ISR must reuse its reserved row across colors, cropped regions,
 // interrupted scans and missing completion. A per-clear malloc/free fails
 // this ownership check and makes SRAM placement depend on unrelated heaps.
 uint32_t clear_words[6];memset(clear_words,0xCD,sizeof(clear_words));
 uint8_t* clear_row=(uint8_t*)(clear_words+1);
 ctx.static_line_buffer=clear_row;clearing=true;
 ctx.area=(EpdRect){4,2,52,679};
 for(unsigned t=0;t<sizeof(scenarios)/sizeof(*scenarios);++t) {
  test_fault=scenarios[t];ctx.error=0;epd_metrics_reset();
  int color=t%3;clear_drive=color==0 ? 1 : color==1 ? 2 : 0;
  epd_push_pixels_lcd(&ctx,120,color);epd_lcd_abort_frame();
  assert(ctx.static_line_buffer==clear_row);
  assert(clear_words[0]==0xCDCDCDCD && clear_words[5]==0xCDCDCDCD);
  assert((ctx.error!=0)==(test_fault!=0));
  EpdMetrics m=epd_metrics_get();assert(m.faults==(test_fault!=0));
  if(test_fault){assert(m.fault_phase==1 && m.fault_line==16 && m.completed==0);}
 }
 assert(atomic_load(&overlap)>0);atomic_store(&quit,true);
 for(int i=0;i<2;++i){xTaskNotifyGive(&tasks[i]);pthread_join(tasks[i].thread,0);lq_free(&ctx.line_queues[i]);vSemaphoreDelete(tasks[i].notify);vSemaphoreDelete(ctx.feed_done_smphr[i]);vSemaphoreDelete(ctx.feed_space[i]);}
 vSemaphoreDelete(ctx.prefill_done);vSemaphoreDelete(ctx.frame_done);free(ctx.line_ready);free(ctx.line_threads);free(ctx.line_mask);free(ctx.conversion_lut);free(data);
 puts("LCD pipeline: phase pixels, queues, persistent clear rows, fault metrics and recovery passed");
}
