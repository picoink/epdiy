#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "output_lcd/lcd_line_layout.h"

int main(void) {
    enum { BYTES=304, ROWS=684 };
    for(int bus=8;bus<=16;bus+=8) {
        const int dummy=epd_lcd_dummy_bytes(bus);
#if defined(CONFIG_IDF_TARGET_ESP32S31)
        assert(dummy==0);
#else
        assert(dummy==bus/8);
#endif
        const size_t stride=BYTES+dummy,size=stride*ROWS;
        uint8_t* allocation=malloc(size+2);assert(allocation);
        memset(allocation,0,size+2);allocation[0]=0x93;allocation[size+1]=0x59;
        uint8_t* wire=allocation+1;
        for(int row=0;row<ROWS;++row) {
            uint8_t* data=wire+epd_lcd_row_offset(row,BYTES,dummy);
            for(int col=0;col<BYTES;++col)data[col]=1+(row*31+col*7)%255;
        }
        for(int row=0;row<ROWS;++row) {
            // Independent receiver: native S31 RGB consumes only panel data;
            // the legacy 8-bit stream starts after its one-byte dummy slot.
#if defined(CONFIG_IDF_TARGET_ESP32S31)
            const uint8_t* received=wire+(size_t)row*BYTES;
#else
            const uint8_t* received=wire+(size_t)row*(BYTES+bus/8)+(bus==8);
#endif
            for(int col=0;col<BYTES;++col)assert(received[col]==1+(row*31+col*7)%255);
        }
        assert(allocation[0]==0x93 && allocation[size+1]==0x59);
        free(allocation);
    }
    return 0;
}
