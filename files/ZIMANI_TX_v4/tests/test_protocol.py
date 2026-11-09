"""Exercise actual firmware C through ctypes. No hardware access; needs native cc."""
import ctypes as C
import dataclasses,json,os,random,subprocess,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pc'))
from zimani_protocol import Command,encode,crc16,parse_status
class Cmd(C.Structure):
    _fields_=[('forward',C.c_int16),('turn',C.c_int16)]+[(k,C.c_uint8) for k in ('blade','cruise','light','engine','engine_adjust','estop')]
class Control(C.Structure):
    _fields_=[(k,C.c_uint32) for k in ('session','seq','last_host_us','last_link_us','accepted','rejected')]+[(k,C.c_uint8) for k in ('host_seen','link_seen','armed','can_arm','fault')]+[('command',Cmd)]
TMP=tempfile.TemporaryDirectory();LIB=Path(TMP.name)/'control.so'
subprocess.run([os.environ.get('CC','cc'),'-shared','-fPIC','-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/zimani_control.c'),'-o',str(LIB)],check=True)
z=C.CDLL(str(LIB));z.z_accept.argtypes=[C.POINTER(Control),C.c_char_p,C.c_uint32];z.z_accept.restype=C.c_int
z.z_init.argtypes=[C.POINTER(Control)];z.z_link.argtypes=[C.POINTER(Control),C.c_uint32];z.z_tick.argtypes=[C.POINTER(Control),C.c_uint32]
z.z_stop.argtypes=[C.POINTER(Control),C.c_uint8];z.z_payload.argtypes=[C.POINTER(Cmd),C.c_void_p]
z.z_reply_valid.argtypes=[C.c_char_p,C.c_uint32];z.z_reply_valid.restype=C.c_int
z.z_crc16.argtypes=[C.c_char_p,C.c_uint32];z.z_crc16.restype=C.c_uint16

def payload(c):
    out=C.create_string_buffer(26);z.z_payload(C.byref(c),out);return out.raw
ACTIVE=Command(armed=True,estop=False)
RELEASE=Command(estop=False,reset_stop=True)
class Tests(unittest.TestCase):
    def setUp(self):self.c=Control();z.z_init(C.byref(self.c));self.seq=0
    def send(self,cmd=Command(),now=1000,session=7):
        self.seq=(self.seq+1)&0xffffffff
        return z.z_accept(C.byref(self.c),encode(session,self.seq,cmd),now)
    def release(self,now=1000):
        self.send(now=now);z.z_link(C.byref(self.c),now);self.send(RELEASE,now=now)
        self.assertEqual((self.c.command.estop,self.c.can_arm),(0,1))
    def arm(self,now=1000):self.release(now);self.send(ACTIVE,now=now);self.assertTrue(self.c.armed)
    def stopped(self):
        c=self.c.command
        self.assertEqual((self.c.armed,self.c.can_arm,c.estop,c.engine,c.engine_adjust,c.forward,c.turn,c.blade,c.cruise,c.light),(0,0,1,2,0,0,0,0,0,0))
    def test_boot_stop_payload(self):
        self.stopped();self.assertEqual(payload(self.c.command).hex().upper(),'AAF8C42000434E1D04020000000064000000000000010001A0ED')
    def test_crc_python_c(self):
        self.assertEqual(crc16(b'123456789'),0x29B1);self.assertEqual(z.z_crc16(b'123456789',9),0x29B1)
        for n in range(100):
            f=encode(3,n,dataclasses.replace(ACTIVE,forward=n,turn=-n,engine=0,engine_adjust=n%3))
            self.assertEqual(len(f),22);self.assertEqual(z.z_crc16(f,20),int.from_bytes(f[20:],'little'))
    def test_new_session_requires_stop(self):
        self.assertFalse(self.send(ACTIVE));self.assertFalse(self.send(RELEASE));self.stopped()
    def test_arm_never_implicitly_resets_stop(self):
        self.send();z.z_link(C.byref(self.c),1000)
        for _ in range(5):self.send(ACTIVE)
        self.stopped()
    def test_release_requires_link(self):self.send();self.send(RELEASE);self.stopped()
    def test_release_then_arm(self):
        self.release();self.send(Command(estop=False));self.assertFalse(self.c.armed)
        self.send(ACTIVE);self.assertTrue(self.c.armed);self.assertFalse(self.c.command.estop)
    def test_reset_must_be_neutral(self):
        for changes in ({'forward':1},{'turn':1},{'blade':1},{'cruise':True},{'light':True},{'engine':0},{'engine_adjust':1},{'armed':True}):
            self.setUp();self.send();z.z_link(C.byref(self.c),1000);self.send(dataclasses.replace(RELEASE,**changes));self.stopped()
    def test_arm_must_be_neutral(self):
        for changes in ({'forward':1},{'turn':1},{'blade':1},{'cruise':True},{'light':True},{'engine':0},{'engine_adjust':1}):
            self.setUp();self.release();self.send(dataclasses.replace(ACTIVE,**changes));self.stopped()
    def test_stop_dominates_every_other_valid_field(self):
        self.arm();self.send(Command(armed=True,estop=True,reset_stop=True,forward=255,turn=-255,blade=2,cruise=True,light=True,engine=0,engine_adjust=2));self.stopped()
    def test_drive_and_engine_adjust(self):
        self.arm();self.send(dataclasses.replace(ACTIVE,engine=0))
        self.send(dataclasses.replace(ACTIVE,engine=0,engine_adjust=1,forward=123,turn=-55,blade=2,cruise=True,light=True))
        p=payload(self.c.command);self.assertEqual((p[9],p[10],p[11],p[12],p[13],p[15],p[17],p[18],p[23]),(0,1,5,123,55,2,1,1,0))
        self.send(dataclasses.replace(ACTIVE,engine=0,engine_adjust=2));self.assertEqual(self.c.command.engine_adjust,2)
        self.send(dataclasses.replace(ACTIVE,engine=0));self.assertEqual(self.c.command.engine_adjust,0)
    def test_engine_change_under_load_latches_stop(self):
        self.arm();self.send(dataclasses.replace(ACTIVE,forward=100));self.send(dataclasses.replace(ACTIVE,engine=0));self.stopped()
    def test_engine_change_new_non_neutral_latches_stop(self):
        self.arm();self.send(dataclasses.replace(ACTIVE,engine=0,forward=100));self.stopped()
    def test_adjust_with_engine_off_latches_stop(self):self.arm();self.send(dataclasses.replace(ACTIVE,engine_adjust=1));self.stopped()
    def test_disarm_latches_stop(self):self.arm();self.send(Command(estop=False));self.stopped()
    def test_reset_while_armed_latches_stop(self):self.arm();self.send(RELEASE);self.stopped()
    def test_host_timeout_no_resume(self):
        self.arm();z.z_link(C.byref(self.c),251000);z.z_tick(C.byref(self.c),251000);self.stopped()
        self.send(ACTIVE,now=251001);self.send(Command(estop=False),now=251002);self.send(ACTIVE,now=251003);self.stopped()
        self.send(RELEASE,now=251004);self.send(ACTIVE,now=251005);self.assertTrue(self.c.armed)
    def test_link_timeout_no_resume(self):
        self.arm();self.send(ACTIVE,now=200000);z.z_tick(C.byref(self.c),301000);self.stopped()
        z.z_link(C.byref(self.c),301001);self.send(ACTIVE,now=301002);self.stopped()
    def test_released_but_unarmed_times_out(self):
        self.release();self.send(Command(estop=False),now=200000);z.z_tick(C.byref(self.c),301000);self.stopped()
    def test_button_radio_uart_stops(self):
        for fault in (4,6,7):
            self.setUp();self.arm();z.z_stop(C.byref(self.c),fault);self.send(ACTIVE);self.stopped()
    def test_stale_and_bad_frames_do_not_refresh(self):
        self.arm();last=self.c.last_host_us
        f=encode(7,self.seq,ACTIVE);self.assertFalse(z.z_accept(C.byref(self.c),f,2000))
        f=bytearray(encode(7,self.seq+1,ACTIVE));f[15]^=4
        self.assertFalse(z.z_accept(C.byref(self.c),bytes(f),2000));self.assertEqual(self.c.last_host_us,last)
    def test_v1_rejected_even_with_valid_crc(self):
        f=bytearray(encode(7,1,Command()));f[2]=1;f[20:]=crc16(f[:20]).to_bytes(2,'little')
        self.assertFalse(z.z_accept(C.byref(self.c),bytes(f),1000));self.stopped()
    def test_invalid_ranges_and_reserved_bits(self):
        for index,value in [(16,3),(17,32),(18,1),(19,3)]:
            f=bytearray(encode(7,1,Command()));f[index]=value;f[20:]=crc16(f[:20]).to_bytes(2,'little')
            self.assertFalse(z.z_accept(C.byref(self.c),bytes(f),1000))
        for n in [-256,256]:
            f=bytearray(encode(7,1,Command()));f[12:14]=n.to_bytes(2,'little',signed=True);f[20:]=crc16(f[:20]).to_bytes(2,'little')
            self.assertFalse(z.z_accept(C.byref(self.c),bytes(f),1000))
    def test_wraparound(self):
        self.seq=0xfffffff9;self.arm(now=0xffffff00)
        for _ in range(6):self.send(ACTIVE,now=0x10)
        self.assertTrue(self.c.armed);z.z_tick(C.byref(self.c),250016);self.stopped()
    def test_new_session_stop_cannot_auto_release(self):
        self.arm();self.assertFalse(self.send(RELEASE,session=8));self.send(session=8);self.stopped()
        self.send(ACTIVE,session=8);self.stopped()
    def test_all_old_and_new_capture_vectors(self):
        old=json.loads((ROOT/'tests/capture_vectors.json').read_text());new=json.loads((ROOT/'tests/capture_vectors_v2.json').read_text())
        for row in old['tx']:
            ref,*v=row;self.assertEqual(payload(Cmd(*v,0,0)),bytes.fromhex(ref))
        for row in new['tx']:
            ref,*v=row;self.assertEqual(payload(Cmd(*v)),bytes.fromhex(ref))
        for ref in old['rx']+new['rx']:
            b=bytes.fromhex(ref);self.assertTrue(z.z_reply_valid(b,len(b)))
            corrupt=bytearray(b);corrupt[11]^=1;self.assertFalse(z.z_reply_valid(bytes(corrupt),len(corrupt)))
        self.assertEqual((len(old['tx']),len(new['tx']),len(old['rx']),len(new['rx'])),(1399,866,1379,741))
        print('Exact TX vectors: 1399 old + 803 clean new + 63 partial prefix = 2265. Valid RX: 2120; corruptions rejected.')
    def test_random_state_invariants(self):
        rng=random.Random(423)
        for i in range(3000):
            now=i*20000
            if rng.random()<.8:z.z_link(C.byref(self.c),now)
            cmd=Command(forward=rng.choice([0,0,1,-255]),turn=rng.choice([0,0,255]),blade=rng.randrange(3),cruise=bool(rng.getrandbits(1)),light=bool(rng.getrandbits(1)),armed=bool(rng.getrandbits(1)),engine=rng.choice([0,2]),engine_adjust=rng.randrange(3),estop=rng.random()<.2,reset_stop=rng.random()<.1)
            if i%11==0:cmd=Command()
            if i%13==0:cmd=RELEASE
            if i%17==0:cmd=ACTIVE
            self.send(cmd,now=now);z.z_tick(C.byref(self.c),now)
            c=self.c.command
            if c.estop:self.stopped()
            if not self.c.armed:self.assertEqual((c.forward,c.turn,c.blade,c.cruise,c.light,c.engine,c.engine_adjust),(0,0,0,0,0,2,0))
            if self.c.armed:self.assertFalse(c.estop)

if __name__=='__main__':unittest.main(verbosity=2)
