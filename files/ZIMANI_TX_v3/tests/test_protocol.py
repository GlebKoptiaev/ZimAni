"""Host-run tests of the ACTUAL firmware control C, linked as a shared library.
Run from package root: python tests/test_protocol.py
Requires a native C compiler (cc or CC). No hardware is accessed.
The included capture_vectors.json contains payloads from all 30 recordings.
"""
import ctypes as C
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from zimani_protocol import Command, encode, crc16, parse_status

class Cmd(C.Structure):
    _fields_=[('forward',C.c_int16),('turn',C.c_int16),('blade',C.c_uint8),('cruise',C.c_uint8),('light',C.c_uint8),('p9',C.c_uint8)]
class Control(C.Structure):
    _fields_=[(k,C.c_uint32) for k in ('session','seq','last_host_us','last_link_us','accepted','rejected')]+[(k,C.c_uint8) for k in ('host_seen','link_seen','armed','can_arm','fault')]+[('command',Cmd)]
TMP=tempfile.TemporaryDirectory()
LIB=Path(TMP.name)/'control.so'
subprocess.run([os.environ.get('CC','cc'),'-shared','-fPIC','-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/zimani_control.c'),'-o',str(LIB)],check=True)
z=C.CDLL(str(LIB))
z.z_accept.argtypes=[C.POINTER(Control),C.c_char_p,C.c_uint32];z.z_accept.restype=C.c_int
z.z_init.argtypes=[C.POINTER(Control)];z.z_link.argtypes=[C.POINTER(Control),C.c_uint32]
z.z_tick.argtypes=[C.POINTER(Control),C.c_uint32];z.z_stop.argtypes=[C.POINTER(Control),C.c_uint8]
z.z_crc16.argtypes=[C.c_char_p,C.c_uint32];z.z_crc16.restype=C.c_uint16
z.z_payload.argtypes=[C.POINTER(Cmd),C.c_void_p];z.z_reply_valid.argtypes=[C.c_char_p,C.c_uint32];z.z_reply_valid.restype=C.c_int

def payload(cmd):
    out=C.create_string_buffer(26);z.z_payload(C.byref(cmd),out);return out.raw

class Tests(unittest.TestCase):
    def setUp(self):
        self.c=Control();z.z_init(C.byref(self.c));self.seq=0
    def send(self,cmd=Command(),now=1000,session=7):
        self.seq=(self.seq+1)&0xFFFFFFFF
        return z.z_accept(C.byref(self.c),encode(session,self.seq,cmd),now)
    def arm(self,now=1000):
        self.send(now=now);z.z_link(C.byref(self.c),now)
        self.send(Command(armed=True),now=now);self.assertEqual(self.c.armed,1)
    def test_crc_known_vector_and_python_c(self):
        self.assertEqual(crc16(b'123456789'),0x29B1)
        self.assertEqual(z.z_crc16(b'123456789',9),0x29B1)
        for f in range(-255,256,17):
            b=encode(7,44,Command(forward=f,turn=-f,armed=True,cruise=True,light=True,blade=2))
            self.assertEqual(len(b),22)
            self.assertEqual(z.z_crc16(b,20),int.from_bytes(b[20:],'little'))
    def test_boot_rejects_armed_new_session(self):
        self.assertEqual(self.send(Command(armed=True)),0);self.assertFalse(self.c.armed)
    def test_arm_requires_radio_and_neutral(self):
        self.send();self.send(Command(armed=True));self.assertFalse(self.c.armed)
        self.send();z.z_link(C.byref(self.c),1000)
        self.send(Command(armed=True,forward=1));self.assertFalse(self.c.armed)
        self.send(Command(armed=True));self.assertFalse(self.c.armed)
        self.send();self.send(Command(armed=True));self.assertTrue(self.c.armed)
    def test_commands_roundtrip_and_disarm(self):
        self.arm();self.send(Command(forward=-123,turn=55,blade=2,cruise=True,light=True,armed=True))
        c=self.c.command
        self.assertEqual((c.forward,c.turn,c.blade,c.cruise,c.light),(-123,55,2,1,1))
        p=payload(c);self.assertEqual(p[11:14],bytes([10,123,55]));self.assertEqual(p[24],sum(p[:24])&255)
        self.send();self.assertFalse(self.c.armed)
        self.assertEqual((c.forward,c.turn,c.blade,c.cruise,c.light),(0,0,0,0,0))
    def test_host_timeout_and_no_automatic_resume(self):
        self.arm();self.send(Command(armed=True,forward=100),now=2000)
        z.z_link(C.byref(self.c),251000);z.z_tick(C.byref(self.c),252000)
        self.assertFalse(self.c.armed);self.assertEqual(self.c.fault,2)
        self.send(Command(armed=True),now=252001);self.assertFalse(self.c.armed)
        self.send(now=252002);self.send(Command(armed=True),now=252003);self.assertTrue(self.c.armed)
    def test_link_timeout_while_host_alive(self):
        self.arm();self.send(Command(armed=True,forward=100),now=200000)
        z.z_tick(C.byref(self.c),301000)
        self.assertFalse(self.c.armed);self.assertEqual(self.c.fault,3)
        z.z_link(C.byref(self.c),301001)
        self.send(Command(armed=True),now=301002);self.assertFalse(self.c.armed)
    def test_bad_crc_duplicate_and_old_sequence_do_not_refresh(self):
        self.send(now=1000);f=encode(7,self.seq,Command())
        self.assertFalse(z.z_accept(C.byref(self.c),f,2000))
        self.assertFalse(z.z_accept(C.byref(self.c),encode(7,0,Command()),3000))
        bad=bytearray(encode(7,10,Command()));bad[13]^=1
        self.assertFalse(z.z_accept(C.byref(self.c),bytes(bad),4000))
        self.assertEqual(self.c.last_host_us,1000)
    def test_counter_wrap(self):
        self.seq=0xFFFFFFFD;self.send(now=0xFFFFFF00)
        z.z_link(C.byref(self.c),0xFFFFFF00)
        self.send(Command(armed=True),now=0xFFFFFF01)
        self.send(Command(armed=True,forward=1),now=0x10)
        self.assertTrue(self.c.armed);self.assertEqual(self.c.seq,0)
        z.z_tick(C.byref(self.c),250016);self.assertFalse(self.c.armed)
    def test_other_session_requires_disarmed_frame(self):
        self.arm();self.assertFalse(self.send(Command(armed=True),session=8))
        self.assertEqual(self.c.session,7)
        self.send(session=8);self.assertFalse(self.c.armed);self.assertEqual(self.c.session,8)
    def test_p9_change_requires_disarm(self):
        self.arm();self.send(Command(armed=True,p9=0));self.assertFalse(self.c.armed)
        self.send(Command(p9=0));self.send(Command(armed=True,p9=0));self.assertTrue(self.c.armed)
        self.assertEqual(payload(self.c.command)[9],0)
    def test_out_of_range_rejected_even_valid_crc(self):
        for offset,value in [(16,3),(17,8),(18,1),(19,1)]:
            b=bytearray(encode(7,1,Command()));b[offset]=value;b[20:]=crc16(b[:20]).to_bytes(2,'little')
            self.assertFalse(z.z_accept(C.byref(self.c),bytes(b),1))
        for val in [-256,256]:
            b=bytearray(encode(7,1,Command()));b[12:14]=val.to_bytes(2,'little',signed=True);b[20:]=crc16(b[:20]).to_bytes(2,'little')
            self.assertFalse(z.z_accept(C.byref(self.c),bytes(b),1))
    def test_payload_known_neutral(self):
        expected=bytes.fromhex('AA F8 C4 20 00 43 4E 1D 04 02 00 00 00 00 64 00 00 00 00 00 00 01 00 00 9F ED')
        self.assertEqual(payload(Cmd(0,0,0,0,0,2)),expected)
    def test_status_parse(self):
        s='S v=3 session=7 seq=2 armed=0 radio=1 fault=0 host_ms=0 link_ms=1 ack=3 fail=0 rx=2 f=0 t=0 reply=BB'
        self.assertEqual(parse_status(s)['ack'],3)
        self.assertIsNone(parse_status('READY'));self.assertIsNone(parse_status('S v=3'))
    def test_all_original_captures(self):
        data=json.loads((ROOT/'tests/capture_vectors.json').read_text())
        tx=rx=0
        for row in data['tx']:
            reference,*values=row
            self.assertEqual(payload(Cmd(*values)),bytes.fromhex(reference),f'TX index {tx}')
            tx+=1
        for raw in data['rx']:
            p=bytes.fromhex(raw);self.assertTrue(z.z_reply_valid(p,len(p)))
            damaged=bytearray(p);damaged[10]^=1;self.assertFalse(z.z_reply_valid(bytes(damaged),23));rx+=1
        self.assertEqual((tx,rx),(1399,1379))
        print(f'Original captures: {tx} TX exact matches; {rx} RX checksums accepted and corruptions rejected.')

if __name__=='__main__':unittest.main(verbosity=2)
