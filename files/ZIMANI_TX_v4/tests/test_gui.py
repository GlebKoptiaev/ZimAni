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

def status(link,armed=0,seq=None,estop=1,can_arm=0,engine=2):
    return (f'S v=4 session={link.session} seq={max(0,link.sequence-1) if seq is None else seq} '
            f'armed={armed} radio=1 fault=0 host_ms=1 link_ms=2 ack=22 fail=0 rx=22 f=0 t=0 '
            f'blade=0 cruise=0 light=0 engine={engine} adjust=0 estop={estop} can_arm={can_arm} stop_rx=0 rx_ms=2 reply=-')

class Tests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk();self.app=App(self.root);self.root.update();self.root.focus_force()
        self.cancel_timers();self.link=FakeLink();self.app.link=self.link
        self.link.lines=[status(self.link)];self.app.receive()
    def cancel_timers(self):
        for timer in self.root.tk.call('after','info'):self.root.after_cancel(timer)
    def tearDown(self):
        self.app.close()
    def release(self):
        self.app.release_stop();self.assertTrue(self.app.release_pending)
        self.link.lines=[status(self.link,estop=0,can_arm=1)];self.app.receive();self.assertFalse(self.app.estop);self.assertFalse(self.app.release_pending)
    def arm(self):
        self.release();self.app.arm();self.assertTrue(self.app.armed);self.assertFalse(self.app.drive_ready)
        self.link.lines=[status(self.link,1,estop=0)];self.app.receive();self.assertTrue(self.app.drive_ready)
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
        self.release();self.link.sequence=10;self.app.arm()
        self.link.lines=[status(self.link,0,seq=9,estop=0,can_arm=1)];self.app.receive();self.assertTrue(self.app.armed)
        self.link.lines=[status(self.link,0,seq=10)];self.app.receive();self.assertFalse(self.app.armed)
    def test_event_loop_pause_disarms(self):
        self.arm();self.app.last_tick=time.monotonic()-0.3;self.app.tick();self.cancel_timers();self.assertFalse(self.app.armed)
    def test_close_port_even_if_write_fails(self):
        self.link.fail=True;self.app.disconnect();self.assertTrue(self.link.closed);self.assertIsNone(self.app.link)


    def test_arm_cannot_clear_stop(self):
        self.app.arm();self.assertFalse(self.app.armed);self.assertTrue(self.app.estop)
    def test_reset_not_repeated_in_heartbeat(self):
        self.app.release_stop();self.assertTrue(self.link.sent[-1].reset_stop)
        self.app.last_tick=time.monotonic();self.app.tick();self.cancel_timers()
        self.assertFalse(self.link.sent[-1].reset_stop);self.assertFalse(self.app.armed)
    def test_delayed_release_status_ignored(self):
        self.link.sequence=10;self.app.release_stop()
        self.link.lines=[status(self.link,seq=10)];self.app.receive();self.assertTrue(self.app.release_pending)
        self.link.lines=[status(self.link,seq=11,estop=0,can_arm=1)];self.app.receive();self.assertFalse(self.app.release_pending)
    def test_release_rejection_latches_local_stop(self):
        self.app.release_stop();self.link.lines=[status(self.link)];self.app.receive()
        self.assertTrue(self.app.estop);self.assertFalse(self.app.release_pending);self.assertTrue(self.link.sent[-1].estop)
    def test_stop_cancels_pending_reset(self):
        self.app.release_stop();self.app.stop()
        self.link.lines=[status(self.link,estop=0,can_arm=1)];self.app.receive()
        self.assertTrue(self.app.estop);self.assertFalse(self.app.armed)
    def test_stale_armed_status_cannot_enable_controls(self):
        self.release();self.link.sequence=10;self.app.arm()
        self.link.lines=[status(self.link,1,seq=9,estop=0)];self.app.receive()
        self.assertFalse(self.app.drive_ready)
    def test_engine_button_and_hold_release(self):
        self.arm();self.app.set_engine(0);self.app.set_adjust(1)
        self.app.last_tick=time.monotonic();self.app.tick();self.cancel_timers()
        c=self.link.sent[-1];self.assertEqual((c.engine,c.engine_adjust,c.estop),(0,1,False))
        self.app.set_adjust(0);self.assertEqual(self.app.engine_adjust,0)
        self.app.set_adjust(2);self.app.stop()
        c=self.link.sent[-1];self.assertEqual((c.engine,c.engine_adjust,c.estop),(2,0,True))
    def test_engine_change_rejected_by_gui_while_moving(self):
        self.arm();self.app.forward.set(5);self.app.set_engine(0);self.assertEqual(self.app.engine,2)
    def test_plus_not_available_when_engine_off(self):
        self.arm();self.app.set_adjust(1);self.assertEqual(self.app.engine_adjust,0)
    def test_released_state_status_timeout_latches_stop(self):
        self.release();self.app.last_status_at=time.monotonic()-1
        self.app.last_tick=time.monotonic();self.app.tick();self.cancel_timers();self.assertTrue(self.app.estop)
    def test_wrong_firmware_version_stops(self):
        self.arm();self.link.lines=[status(self.link).replace('v=4','v=3')];self.app.receive();self.assertTrue(self.app.estop)
    def test_board_restart_stops(self):
        self.arm();self.link.lines=[status(self.link).replace('session=123','session=0')];self.app.receive();self.assertTrue(self.app.estop)
    def test_full_gui_to_actual_c(self):
        from test_protocol import Control,z,C
        from zimani_protocol import encode
        c=Control();z.z_init(C.byref(c));link=self.link
        def send(command):
            seq=link.sequence;link.sequence+=1;link.sent.append(command)
            self.assertTrue(z.z_accept(C.byref(c),encode(link.session,seq,command),1000));return seq
        link.send=send
        link.send(__import__('zimani_protocol').Command());z.z_link(C.byref(c),1000)
        def feedback():
            link.lines=[status(link,armed=c.armed,estop=c.command.estop,can_arm=c.can_arm,engine=c.command.engine)];self.app.receive()
        feedback();self.app.release_stop();feedback();self.app.arm();feedback()
        self.app.set_engine(0);self.app.last_tick=time.monotonic();self.app.tick();self.cancel_timers()
        self.assertEqual(c.command.engine,0)
        self.app.set_adjust(2);self.app.last_tick=time.monotonic();self.app.tick();self.cancel_timers()
        self.assertEqual(c.command.engine_adjust,2)
        self.app.stop();self.assertEqual((c.armed,c.command.estop,c.command.engine_adjust,c.command.engine),(0,1,0,2))

if __name__=='__main__':unittest.main(verbosity=2)

