#!/usr/bin/env python3
"""Build without HAL/libc, using CMSIS + startup from YOUR CubeMX F429 project.
python standalone/build.py --project C:/STM32/MyCubeProject --gcc C:/.../arm-none-eabi-gcc.exe
Outputs build/ZIMANI_TX_v3.elf and .bin. Does not flash hardware.
"""
import argparse
from pathlib import Path
import shutil
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--project',required=True,type=Path)
p.add_argument('--gcc',default='arm-none-eabi-gcc')
a=p.parse_args();root=Path(__file__).resolve().parents[1];project=a.project.resolve()
cc=shutil.which(a.gcc) or a.gcc
cmsis=project/'Drivers/CMSIS'
include=[root/'Core/Inc',cmsis/'Include',cmsis/'Device/ST/STM32F4xx/Include']
startup=list((project/'Core/Startup').glob('startup_stm32f429*.s'))
if len(startup)!=1:raise SystemExit('Expected exactly one Core/Startup/startup_stm32f429*.s in Cube project')
system=project/'Core/Src/system_stm32f4xx.c'
for path in [*include,system]:
    if not path.exists():raise SystemExit('Missing '+str(path))
build=root/'build';build.mkdir(exist_ok=True)
flags=['-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=c11','-O2','-g3','-ffreestanding','-ffunction-sections','-fdata-sections','-DSTM32F429xx','-Wall','-Wextra','-Werror']+['-I'+str(x) for x in include]
objects=[]
for source in [root/'Core/Src/main.c',root/'Core/Src/zimani_control.c',system,startup[0],root/'standalone/runtime.c']:
    obj=build/(source.stem+'.o');objects.append(str(obj))
    subprocess.run([cc,*flags,'-c',str(source),'-o',str(obj)],check=True)
elf=build/'ZIMANI_TX_v3.elf';binary=build/'ZIMANI_TX_v3.bin'
subprocess.run([cc,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-nostdlib','-T'+str(root/'standalone/stm32f429zi.ld'),'-Wl,--gc-sections,-Map='+str(build/'ZIMANI_TX_v3.map'),*objects,'-lgcc','-o',str(elf)],check=True)
subprocess.run([str(Path(cc).with_name(Path(cc).name.replace('gcc','objcopy'))),'-O','binary',str(elf),str(binary)],check=True)
print('Built:',elf,binary,sep='\n')
