#!/usr/bin/env python3
"""Run the production LCD pipeline/LUT/queues with pthread workers and a fake LCD."""
from pathlib import Path
import os, subprocess, tempfile
here=Path(__file__).resolve().parent
src=here.parents[1]/'src'
with tempfile.TemporaryDirectory(prefix='epd-pipeline-') as tmp:
    for full, psram in ((0,0),(0,1),(1,0),(1,1)):
        exe=Path(tmp)/'pipeline'
        subprocess.run([os.environ.get('CC','cc'),'-std=gnu11','-O2','-pthread','-g','-fsanitize=undefined','-fno-sanitize-recover=all',
            '-Wall','-Wextra','-Wno-unknown-attributes','-Wno-unused-parameter',f'-DEPD_LINE_QUEUE_PSRAM={psram}',f'-DEPD_LCD_FULL_PHASE={full}',
            '-I'+str(here/'stubs'),'-I'+str(src),str(here/'pipeline.c'),
            *(str(src/p) for p in ('output_lcd/render_lcd.c','output_common/render_context.c','output_common/line_queue.c','output_common/lut.c','epd_metrics.c','output_common/riscv_ve.c')),
            '-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=30)
