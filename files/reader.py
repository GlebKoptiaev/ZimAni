#!/usr/bin/env python3
"""Record F429 UART output and decode nRF24 SPI transactions (Python 3.9+)."""
import argparse
import csv
from pathlib import Path
import re
import sys
import time

REG = {0x00:'CONFIG', 0x01:'EN_AA', 0x02:'EN_RXADDR', 0x03:'SETUP_AW',
       0x04:'SETUP_RETR', 0x05:'RF_CH', 0x06:'RF_SETUP', 0x07:'STATUS',
       0x08:'OBSERVE_TX', 0x09:'RPD', 0x0A:'RX_ADDR_P0', 0x0B:'RX_ADDR_P1',
       0x0C:'RX_ADDR_P2', 0x0D:'RX_ADDR_P3', 0x0E:'RX_ADDR_P4', 0x0F:'RX_ADDR_P5',
       0x10:'TX_ADDR', 0x11:'RX_PW_P0', 0x12:'RX_PW_P1', 0x13:'RX_PW_P2',
       0x14:'RX_PW_P3', 0x15:'RX_PW_P4', 0x16:'RX_PW_P5', 0x17:'FIFO_STATUS',
       0x1C:'DYNPD', 0x1D:'FEATURE'}
FIELDS=['index','end_us','command','status','direction','register','data_hex',
        'MOSI_hex','MISO_hex','note']

def decode_frame(index, stamp, mosi, miso):
    if len(mosi)!=len(miso):
        raise ValueError(f'Frame {index}: MOSI/MISO lengths differ')
    r=dict.fromkeys(FIELDS,'')
    r.update(index=index,end_us=stamp,MOSI_hex=mosi.hex().upper(),
             MISO_hex=miso.hex().upper())
    if not mosi:
        r.update(command='EMPTY',note='CSN pulse without a complete byte')
        return r
    op=mosi[0]
    r['status']=f'{miso[0]:02X}'
    data=b''
    if 0x00<=op<=0x1F:
        r.update(command='R_REGISTER',direction='radio_to_robot',
                 register=REG.get(op,f'0x{op:02X}')); data=miso[1:]
    elif 0x20<=op<=0x3F:
        reg=op&0x1F
        r.update(command='W_REGISTER',direction='robot_to_radio',
                 register=REG.get(reg,f'0x{reg:02X}')); data=mosi[1:]
        if reg==5 and data: r['note']=f'RF_CH={data[0]}; nominal MHz={2400+data[0]}'
    elif op==0x61:
        r.update(command='R_RX_PAYLOAD',direction='radio_to_robot'); data=miso[1:]
    elif op in (0xA0,0xB0):
        r.update(command='W_TX_PAYLOAD' if op==0xA0 else 'W_TX_PAYLOAD_NOACK',
                 direction='robot_to_radio'); data=mosi[1:]
    elif 0xA8<=op<=0xAD:
        r.update(command='W_ACK_PAYLOAD',direction='robot_to_radio',
                 note=f'pipe={op&7}'); data=mosi[1:]
    elif op==0x60:
        r.update(command='R_RX_PL_WID',direction='radio_to_robot'); data=miso[1:]
    elif op==0x50:
        r.update(command='ACTIVATE',direction='robot_to_radio'); data=mosi[1:]
    else:
        r['command']={0xE1:'FLUSH_TX',0xE2:'FLUSH_RX',0xE3:'REUSE_TX_PL',
                      0xFF:'NOP'}.get(op,f'UNKNOWN_0x{op:02X}')
    r['data_hex']=data.hex().upper()
    if len(mosi)>33:
        r['note']+='; more than 33 bytes: check framing/protocol'
    if 'PAYLOAD' in r['command'] and not 1<=len(data)<=32:
        r['note']+='; invalid nRF24 payload length'
    return r

def parse_capture(text, allow_partial=False):
    lines=text.splitlines()
    begin=[i for i,s in enumerate(lines) if s.startswith('#BEGIN ')]
    if len(begin)!=1: raise ValueError('Expected exactly one #BEGIN capture')
    start=begin[0]
    end=next((i for i in range(start+1,len(lines)) if lines[i].strip()=='#END'),None)
    if end is None: raise ValueError('Missing #END: UART export is incomplete')
    meta=dict(re.findall(r'(\w+)=([^\s]+)',lines[start]))
    if meta.get('version')!='1': raise ValueError('Unsupported capture version')
    flags=int(meta.get('flags','-1'),0)
    if flags and not allow_partial:
        raise ValueError(f'Capture flags={flags:#x}; see README. '
                         'Use --allow-partial only to inspect the earlier saved prefix.')
    result=[]
    total=0
    for line in lines[start+1:end]:
        if not line.startswith('F,'): continue
        cells=line.split(',')
        if len(cells)!=5: raise ValueError('Malformed frame line')
        _,i,t,a,b=cells
        if int(i)!=len(result): raise ValueError('Missing or reordered frame')
        mosi,miso=bytes.fromhex(a),bytes.fromhex(b)
        result.append(decode_frame(int(i),int(t),mosi,miso))
        total+=len(mosi)
    if len(result)!=int(meta['frames']) or total!=int(meta['complete_bytes']):
        raise ValueError('Header counts disagree with received data')
    return meta,result

def record(port, output, seconds):
    try: import serial
    except ImportError: raise SystemExit('Install pyserial: python -m pip install pyserial')
    with serial.Serial(port,115200,timeout=0.2,write_timeout=2) as dev:
        time.sleep(0.2)
        dev.reset_input_buffer()
        dev.write(b'a'); dev.flush()
        deadline=time.monotonic()+seconds
        buf=bytearray()
        print('Capture started. Move one control now; waiting for UART export...')
        while time.monotonic()<deadline:
            buf.extend(dev.read(max(1,dev.in_waiting)))
            if b'#END\r\n' in buf or b'#END\n' in buf:
                Path(output).write_bytes(buf)
                print(f'Saved {output} ({len(buf)} bytes)')
                return
        partial=str(output)+'.partial'
        Path(partial).write_bytes(buf)
        raise SystemExit(f'Timeout; incomplete bytes saved in {partial}')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    r=sub.add_parser('record');r.add_argument('--port',required=True)
    r.add_argument('--out',default='capture.txt');r.add_argument('--timeout',type=float,default=60)
    d=sub.add_parser('decode');d.add_argument('input');d.add_argument('--out',default='decoded.csv')
    d.add_argument('--allow-partial',action='store_true')
    a=p.parse_args()
    if a.action=='record': record(a.port,a.out,a.timeout);return
    meta,rows=parse_capture(Path(a.input).read_text(encoding='ascii'),a.allow_partial)
    with open(a.out,'w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader();w.writerows(rows)
    print(f"{len(rows)} frames; stop={meta['reason']}; flags={meta['flags']}; {a.out}")
    print('R_RX_PAYLOAD rows contain packets received by the robot, excluding STATUS.')
    print('Payload field meanings (joysticks/buttons/checksums) remain to be determined.')

if __name__=='__main__':
    try: main()
    except (ValueError,OSError) as e:
        print(f'Error: {e}',file=sys.stderr);sys.exit(1)
