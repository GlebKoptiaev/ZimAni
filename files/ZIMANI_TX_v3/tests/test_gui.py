"""Tk UI tests with a simulated link; no serial port or robot is opened.
Run with a desktop/display: python tests/test_gui.py
Linux headless: start Xvfb and set DISPLAY. Requires pyserial and Tkinter.
"""
from pathlib import Path
import sys,time,unittest
import tkinter as tk
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pc'))
from zimani_control import App

class FakeLink:
    def __init__(self):
        self.session=123;self.sequence=0;self.sent=[];self.lines=[];self.closed=False;self.fail=False
    def send(self,c):
        if self.fail:raise OSError('simulated unplug')
        self.sent.append(c);s=self.sequence;self.sequence=(s+1)&0xFFFFFFFF;return s
    def read_lines(self):
        out=self.lines;self.lines=[];return out
    def close(self):self.closed=True

def status(link,armed=0,seq=None):
    return (f'S v=3 session={link.session} seq={max(0,link.sequence-1) if seq is None else seq} '
            f'armed={armed} radio=1 fault=0 host_ms=1 link_ms=2 ack=22 fail=0 rx=22 f=0 t=0 '
            'blade=0 cruise=0 light=0 p9=2 reply=-')

class Tests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk();self.app=App(self.root);self.root.update();self.root.focus_force()
        self.cancel_timers();self.link=FakeLink();self.app.link=self.link
        self.link.lines=[status(self.link)];self.app.receive()
    def cancel_timers(self):
        for timer in self.root.tk.call('after','info'):self.root.after_cancel(timer)
    def tearDown(self):
        self.app.close()
    def arm(self):
        self.app.arm();self.assertTrue(self.app.armed);self.assertFalse(self.app.drive_ready)
        self.link.lines=[status(self.link,1)];self.app.receive();self.assertTrue(self.app.drive_ready)
    def test_arm_waits_for_board_and_sends_controls(self):
        self.arm();a=self.app;a.forward.set(100);a.turn.set(-60);a.set_blade(2);a.cruise.set(True);a.light.set(True)
        a.last_tick=time.monotonic();a.tick();self.cancel_timers();c=self.link.sent[-1]
        self.assertEqual((c.forward,c.turn,c.blade,c.cruise,c.light,c.armed),(100,-60,2,True,True,True))
        a.stop();self.assertEqual((a.forward.get(),a.turn.get(),a.blade),(0,0,0));self.assertFalse(self.link.sent[-1].armed)
    def test_center_disabled_sliders(self):
        a=self.app;a.set_controls(True);a.forward.set(200);a.turn.set(-100);a.set_controls(False)
        a.center();self.assertEqual((a.forward.get(),a.turn.get()),(0,0))
    def test_no_fresh_status_no_arm(self):
        self.app.last_status_at=0;self.app.arm();self.assertFalse(self.app.armed)
    def test_board_disarm_requires_new_arm(self):
        self.arm();self.link.lines=[status(self.link,0)];self.app.receive();self.assertFalse(self.app.armed)
    def test_old_disarmed_status_does_not_cancel_pending_arm(self):
        self.link.sequence=10;self.app.arm()
        self.link.lines=[status(self.link,0,seq=9)];self.app.receive();self.assertTrue(self.app.armed)
        self.link.lines=[status(self.link,0,seq=10)];self.app.receive();self.assertFalse(self.app.armed)
    def test_event_loop_pause_disarms(self):
        self.arm();self.app.last_tick=time.monotonic()-0.3;self.app.tick();self.cancel_timers();self.assertFalse(self.app.armed)
    def test_close_port_even_if_write_fails(self):
        self.link.fail=True;self.app.disconnect();self.assertTrue(self.link.closed);self.assertIsNone(self.app.link)

if __name__=='__main__':unittest.main(verbosity=2)
