#!/usr/bin/env python3
"""Real-time desktop controller for NUCLEO-F429ZI ZimAni TX v4.
Install: python -m pip install -r requirements.txt
Run: python zimani_control.py
"""
import argparse
from datetime import datetime
from pathlib import Path
import secrets
import time
import tkinter as tk
from tkinter import ttk, messagebox

from zimani_protocol import Command, encode, parse_status

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit('Установите pyserial: python -m pip install pyserial')

FAULTS = {0:'нет',1:'после сброса',2:'нет команд Python',3:'нет связи по радио',
          4:'ошибка радиомодуля/SPI',5:'требуется нейтраль и повторное разрешение',
          6:'ошибка UART',7:'нажата кнопка B1 на Nucleo',8:'STOP активен'}

class Link:
    def __init__(self, port):
        self.port = serial.Serial(port, 115200, timeout=0, write_timeout=0.05,
                                  rtscts=False, dsrdtr=False)
        self.session = secrets.randbits(32) or 1
        self.sequence = 0
        self.buffer = bytearray()

    def send(self, command):
        seq = self.sequence
        frame = encode(self.session, seq, command)
        if self.port.write(frame) != len(frame):
            raise serial.SerialTimeoutException('Не весь пакет записан')
        self.sequence = (seq + 1) & 0xFFFFFFFF
        return seq

    def read_lines(self):
        self.buffer.extend(self.port.read(min(self.port.in_waiting, 4096)))
        lines = []
        while b'\n' in self.buffer:
            line, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            lines.append(line.decode('ascii', errors='replace').strip())
        if len(self.buffer) > 8192:
            self.buffer.clear()
        return lines

    def close(self):
        self.port.close()

class App:
    def __init__(self, root, initial_port='', log_path=None):
        self.root = root
        root.title('ZimAni — управление через STM32 v4')
        root.geometry('1000x890')
        root.minsize(960,850)
        self.link = None
        self.estop = True
        self.release_pending = False
        self.release_sequence = 0
        self.release_time = 0.0
        self.armed = False
        self.drive_ready = False
        self.arm_time = 0.0
        self.arm_sequence = 0
        self.last_status = None
        self.last_status_at = 0.0
        self.last_tick = time.monotonic()
        self.closing = False
        self.log = Path(log_path).open('a', encoding='utf-8', buffering=1) if log_path else None
        self.port_name = tk.StringVar(value=initial_port)
        self.light = tk.BooleanVar()
        self.cruise = tk.BooleanVar()
        self.engine = 2
        self.engine_adjust = 0
        self.engine_label = tk.StringVar(value='ДВС: команда ВЫКЛ (p[9]=2)')
        self.blade = 0
        self.notice = tk.StringVar(value='Подключите COM-порт. Управление выключено.')
        self.status = tk.StringVar(value='Статуса STM32 пока нет')
        top = ttk.Frame(root, padding=10);top.pack(fill='x')
        ttk.Label(top, text='COM-порт:').pack(side='left')
        self.port_combo = ttk.Combobox(top, textvariable=self.port_name, width=18)
        self.port_combo.pack(side='left', padx=5)
        ttk.Button(top, text='Обновить', command=self.refresh).pack(side='left')
        self.connect_button = ttk.Button(top, text='Подключить', command=self.toggle_connection)
        self.connect_button.pack(side='left', padx=8)
        self.arm_button = ttk.Button(top, text='Разрешить управление', command=self.arm, state='disabled')
        self.arm_button.pack(side='right')
        self.release_button = ttk.Button(top, text='Снять STOP', command=self.release_stop, state='disabled')
        self.release_button.pack(side='right', padx=8)
        tk.Button(root, text='STOP — ОСТАНОВКА  [ПРОБЕЛ]', bg='#B42318', fg='white',
                  font=('Segoe UI', 13, 'bold'), command=self.stop).pack(fill='x', padx=10, pady=5)
        ttk.Label(root, textvariable=self.notice, wraplength=860).pack(fill='x', padx=12, pady=4)
        body = ttk.Frame(root, padding=10);body.pack(fill='both', expand=True)
        axes = ttk.LabelFrame(body, text='Джойстик: команда −255…255', padding=10)
        axes.pack(side='left', fill='both', expand=True)
        self.canvas = tk.Canvas(axes, width=330, height=220, bg='#edf5f5', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Button-1>', self.pad)
        self.canvas.bind('<B1-Motion>', self.pad)
        self.canvas.bind('<ButtonRelease-1>', self.pad_release)
        self.canvas.bind('<Configure>', lambda e:self.draw())
        self.forward = tk.Scale(axes, from_=-255, to=255, orient='horizontal', resolution=1,
                                label='Вперёд + / назад −', command=lambda _:self.draw())
        self.forward.pack(fill='x')
        self.turn = tk.Scale(axes, from_=-255, to=255, orient='horizontal', resolution=1,
                             label='Влево − / вправо +', command=lambda _:self.draw())
        self.turn.pack(fill='x')
        ttk.Button(axes, text='Вернуть обе оси в центр', command=self.center).pack(pady=5)
        ttk.Label(axes, text='Ползунки удерживают значения. На поле мышь работает как\n'
                  'джойстик: отпускание возвращает оси в центр.\n'
                  'При круизе поле удерживает продольную команду.', wraplength=430).pack()
        other = ttk.LabelFrame(body, text='Функции', padding=12)
        other.pack(side='right', fill='y', padx=(10,0))
        self.light_widget = ttk.Checkbutton(other, text='Свет (поле p[17])', variable=self.light)
        self.light_widget.pack(anchor='w', pady=5)
        self.cruise_widget = ttk.Checkbutton(other, text='Круиз (поле p[18])', variable=self.cruise)
        self.cruise_widget.pack(anchor='w', pady=5)
        ttk.Label(other, text='Назначение света и круиза\nтребует проверки на вашей технике.', foreground='#855500').pack(anchor='w', pady=8)
        ttk.Label(other, text='Высота ножа / деки:').pack(anchor='w', pady=(15,5))
        self.blade_up = ttk.Button(other, text='ПОДНЯТЬ — удерживать')
        self.blade_up.pack(fill='x', pady=5)
        self.blade_down = ttk.Button(other, text='ОПУСТИТЬ — удерживать')
        self.blade_down.pack(fill='x', pady=5)
        self.blade_up.bind('<ButtonPress-1>', lambda e:self.set_blade(2))
        self.blade_down.bind('<ButtonPress-1>', lambda e:self.set_blade(1))
        root.bind_all('<ButtonRelease-1>', lambda e:self.set_blade(0), add='+')
        ttk.Label(other, text='Отпускание кнопки снимает команду.\nАбсолютная высота не задаётся.', wraplength=280).pack(anchor='w')
        ttk.Separator(other).pack(fill='x', pady=15)
        ttk.Label(other, textvariable=self.engine_label).pack(anchor='w')
        self.engine_on = ttk.Button(other, text='ДВС: команда ВКЛ', command=lambda:self.set_engine(0))
        self.engine_on.pack(fill='x', pady=3)
        self.engine_off = ttk.Button(other, text='ДВС: команда ВЫКЛ', command=lambda:self.set_engine(2))
        self.engine_off.pack(fill='x', pady=3)
        ttk.Label(other, text='Менять только при нейтрали. Это код команды,\nне показание работающего двигателя;\nпоследовательность стартера ещё не установлена.', wraplength=320).pack(anchor='w')
        self.engine_plus = ttk.Button(other, text='ДВИГАТЕЛЬ + — удерживать')
        self.engine_plus.pack(fill='x', pady=4)
        self.engine_minus = ttk.Button(other, text='ДВИГАТЕЛЬ − — удерживать')
        self.engine_minus.pack(fill='x', pady=4)
        self.engine_plus.bind('<ButtonPress-1>', lambda e:self.set_adjust(1))
        self.engine_minus.bind('<ButtonPress-1>', lambda e:self.set_adjust(2))
        root.bind_all('<ButtonRelease-1>', lambda e:self.set_adjust(0), add='+')
        ttk.Label(root, textvariable=self.status, justify='left', background='#e8edf1',
                  padding=10, wraplength=870).pack(side='bottom', before=body, fill='x', padx=10, pady=10)
        root.bind('<space>', lambda e:self.stop())
        root.bind('<FocusOut>', lambda e:root.after_idle(self.check_focus))
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.widgets = [self.forward,self.turn,self.light_widget,self.cruise_widget,self.blade_up,self.blade_down,self.engine_on,self.engine_off,self.engine_plus,self.engine_minus]
        self.set_controls(False)
        self.refresh()
        root.after(20,self.tick)

    def record(self, line):
        if self.log:
            self.log.write(datetime.now().isoformat(timespec='milliseconds')+' '+line+'\n')

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo['values'] = ports
        if not self.port_name.get() and ports:self.port_name.set(ports[0])

    def set_controls(self, enabled):
        for widget in self.widgets:widget.configure(state='normal' if enabled else 'disabled')

    def toggle_connection(self):
        if self.link:
            self.disconnect();return
        try:
            self.link = Link(self.port_name.get())
            self.last_status = None;self.last_status_at=0
            self.link.send(Command())
            self.connect_button.configure(text='Отключить')
            self.arm_button.configure(state='normal');self.release_button.configure(state='normal')
            self.notice.set('Подключено. Дождитесь ответов BB, нажмите «Снять STOP», затем «Разрешить управление».')
        except (OSError,serial.SerialException) as e:
            self.disconnect();messagebox.showerror('COM-порт',str(e))

    def center(self):
        for slider in (self.forward, self.turn):
            state = slider.cget('state')
            slider.configure(state='normal');slider.set(0);slider.configure(state=state)
        self.draw()

    def stop(self, message='STOP защёлкнут. Передаётся p[23]=1, ДВС ВЫКЛ и нулевые оси.'):
        self.estop=True;self.release_pending=False
        self.engine=2;self.engine_adjust=0;self.engine_label.set('ДВС: команда ВЫКЛ (p[9]=2)')
        self.armed=False;self.drive_ready=False;self.center();self.blade=0
        self.cruise.set(False);self.light.set(False);self.set_controls(False)
        self.notice.set(message)
        if self.link:
            try:self.link.send(Command())
            except (OSError,serial.SerialException):pass
        self.record('HOST STOP '+message)

    def disconnect(self):
        self.stop('COM-порт отключён.')
        if self.link:
            try:
                self.link.send(Command())
            except (OSError,serial.SerialException):pass
            finally:
                try:self.link.close()
                except (OSError,serial.SerialException):pass
            self.link=None
        self.arm_button.configure(state='disabled');self.release_button.configure(state='disabled');self.connect_button.configure(text='Подключить')

    def ready_status(self):
        s=self.last_status
        return bool(self.link and s and time.monotonic()-self.last_status_at<=0.6
                    and s['session']==self.link.session and s['radio'] and s['link_ms']<300
                    and s['host_ms']<250)

    def release_stop(self):
        if self.armed or self.release_pending:
            self.notice.set('Сначала нажмите STOP; дождитесь подтверждения предыдущего действия.');return
        if not self.ready_status():
            self.notice.set('Для снятия STOP нужен свежий проверенный ответ BB от робота.');return
        self.stop()
        self.estop=False;self.release_pending=True;self.release_time=time.monotonic()
        try:self.release_sequence=self.link.send(Command(estop=False,reset_stop=True))
        except (OSError,serial.SerialException) as e:self.stop(str(e));return
        self.notice.set('Запрос снятия STOP отправлен. Ожидание STM32…')

    def arm(self):
        if self.estop or self.release_pending:
            self.notice.set('Сначала нажмите «Снять STOP» и дождитесь подтверждения STM32.');return
        if not self.ready_status() or not self.last_status['can_arm'] or self.last_status['estop']:
            self.notice.set('Нужны свежий BB-ответ и снятый STOP. Повторите снятие STOP.');return
        self.center();self.blade=0;self.engine=2;self.engine_adjust=0
        self.cruise.set(False);self.light.set(False)
        self.armed=True;self.drive_ready=False;self.arm_time=time.monotonic()
        self.set_controls(False)
        try:self.arm_sequence=self.link.send(Command(armed=True,estop=False))
        except (OSError,serial.SerialException) as e:self.stop(str(e));return
        self.notice.set('Ожидание подтверждения разрешения от STM32…')

    def set_engine(self, value):
        if not self.drive_ready:return
        s=self.last_status or {}
        if (self.forward.get() or self.turn.get() or self.blade or self.cruise.get()
            or self.engine_adjust or s.get('f',0) or s.get('t',0) or s.get('blade',0)
            or s.get('cruise',0) or s.get('adjust',0)):
            self.notice.set('Перед изменением команды ДВС верните оси в центр, снимите круиз и отпустите кнопки.');return
        self.engine=value
        self.engine_label.set('ДВС: команда '+('ВКЛ (p[9]=0)' if value==0 else 'ВЫКЛ (p[9]=2)'))

    def set_adjust(self, value):
        self.engine_adjust=value if self.drive_ready and self.engine==0 else 0

    def set_blade(self, value):
        self.blade=value if self.drive_ready else 0

    def pad(self, event):
        if not self.drive_ready:return
        w=max(1,self.canvas.winfo_width());h=max(1,self.canvas.winfo_height())
        self.turn.set(round(max(-1,min(1,(event.x-w/2)/(w*.43)))*255))
        self.forward.set(round(max(-1,min(1,(h/2-event.y)/(h*.43)))*255))
        self.draw()

    def pad_release(self, event):
        if not self.cruise.get():self.forward.set(0)
        self.turn.set(0);self.draw()

    def draw(self):
        if not hasattr(self,'turn'):return
        c=self.canvas;w=max(1,c.winfo_width());h=max(1,c.winfo_height());c.delete('all')
        c.create_line(w/2,15,w/2,h-15,fill='#a4b9c2');c.create_line(15,h/2,w-15,h/2,fill='#a4b9c2')
        c.create_text(w/2,12,text='ВПЕРЁД');c.create_text(w/2,h-12,text='НАЗАД')
        x=w/2+self.turn.get()/255*w*.43;y=h/2-self.forward.get()/255*h*.43
        c.create_oval(x-9,y-9,x+9,y+9,fill='#087f8c' if self.drive_ready else '#888888',outline='')
        c.create_text(8,8,anchor='nw',text=f'V={self.forward.get()}  R={self.turn.get()}')

    def check_focus(self):
        if not self.estop and self.root.focus_displayof() is None:
            self.stop('Окно потеряло фокус: управление снято.')

    def receive(self):
        for line in self.link.read_lines():
            self.record(line)
            s=parse_status(line)
            if s is None:
                if line.startswith('S v=') and not line.startswith('S v=4 '):
                    self.stop('Версия прошивки несовместима. Нужна STM32 TX v4 и эта панель.')
                continue
            self.last_status=s;self.last_status_at=time.monotonic()
            fault=FAULTS.get(s['fault'],str(s['fault']))
            self.status.set(f"STM32: radio={s['radio']} armed={s['armed']} STOP={s['estop']} причина: {fault}\n"
                f"ACK={s['ack']} ошибок TX={s['fail']} ответов BB={s['rx']} возраст BB={s['link_ms']} мс\n"
                f"V={s['f']} R={s['t']} ДВС-код={s['engine']} +/−={s['adjust']} r[11]={s['stop_rx']} (возраст {s['rx_ms']} мс)\n"
                f"Последний ответ: {s.get('reply','-')}")
            if s['session']!=self.link.session:
                if not self.estop:self.stop('STM32 перезапустилась / изменилась сессия. STOP защёлкнут.')
                continue
            if self.release_pending:
                if ((s['seq']-self.release_sequence)&0xFFFFFFFF)<0x80000000:
                    if s['estop'] or not s['can_arm']:
                        self.stop('STM32 отклонила снятие STOP: '+fault)
                    else:
                        self.release_pending=False
                        self.notice.set('STOP снят в STM32. Движение ещё запрещено; нажмите «Разрешить управление».')
                continue
            if self.estop:continue
            if self.armed:
                acknowledged=((s['seq']-self.arm_sequence)&0xFFFFFFFF)<0x80000000
                if not acknowledged:continue
                if s['armed'] and not s['estop']:
                    self.drive_ready=True;self.set_controls(True)
                    self.notice.set('УПРАВЛЕНИЕ РАЗРЕШЕНО. Пробел или красная кнопка — STOP.')
                else:self.stop('STM32 сняла/отклонила разрешение: '+fault)
            elif s['estop']:
                self.stop('STM32 установила STOP: '+fault)

    def tick(self):
        if self.closing:return
        now=time.monotonic();gap=now-self.last_tick;self.last_tick=now
        if not self.estop and gap>0.20:self.stop('Пауза Python более 200 мс: требуется повторное разрешение.')
        if self.link:
            try:
                self.receive()
                if not self.estop and (now-self.last_status_at>0.7 or
                    (self.armed and not self.drive_ready and now-self.arm_time>0.7) or
                    (self.release_pending and now-self.release_time>0.7)):
                    self.stop('Нет подтверждения STM32: управление снято.')
                cmd=Command(estop=self.estop)
                if self.armed:
                    cmd=Command(forward=self.forward.get() if self.drive_ready else 0,
                                turn=self.turn.get() if self.drive_ready else 0,
                                blade=self.blade if self.drive_ready else 0,
                                cruise=self.cruise.get() if self.drive_ready else False,
                                light=self.light.get() if self.drive_ready else False,
                                armed=True,estop=False,engine=self.engine if self.drive_ready else 2,
                                engine_adjust=self.engine_adjust if self.drive_ready else 0)
                self.link.send(cmd)
            except (OSError,serial.SerialException) as e:
                self.disconnect();self.notice.set('Ошибка COM-порта: '+str(e))
        self.root.after(20,self.tick)

    def close(self):
        self.closing=True;self.disconnect()
        if self.log:self.log.close()
        self.root.destroy()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',default='');p.add_argument('--log')
    args=p.parse_args();root=tk.Tk();App(root,args.port,args.log);root.mainloop()

if __name__=='__main__':main()
