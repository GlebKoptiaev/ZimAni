#!/usr/bin/env python3
"""Real-time desktop controller for NUCLEO-F429ZI ZimAni TX v3.
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
          6:'ошибка UART',7:'нажата кнопка B1 на Nucleo'}

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
        root.title('ZimAni — управление через STM32 v3')
        root.geometry('900x720')
        root.minsize(850,650)
        self.link = None
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
        self.p9 = tk.StringVar(value='2')
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
        tk.Button(root, text='СТОП / СНЯТЬ РАЗРЕШЕНИЕ  [ПРОБЕЛ]', bg='#B42318', fg='white',
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
        ttk.Label(other, text='Исследуемое поле p[9]:').pack(anchor='w')
        self.p9_widget = ttk.Combobox(other, values=['2','0'], textvariable=self.p9,
                                      state='readonly', width=8)
        self.p9_widget.pack(anchor='w', pady=5)
        ttk.Label(other, text='По умолчанию 2, как в записях\nторможения и диагоналей.\nМенять только при снятом разрешении.', wraplength=270).pack(anchor='w')
        ttk.Label(root, textvariable=self.status, justify='left', background='#e8edf1',
                  padding=10, wraplength=870).pack(side='bottom', before=body, fill='x', padx=10, pady=10)
        root.bind('<space>', lambda e:self.stop())
        root.bind('<FocusOut>', lambda e:root.after_idle(self.check_focus))
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.widgets = [self.forward,self.turn,self.light_widget,self.cruise_widget,self.blade_up,self.blade_down]
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
        self.p9_widget.configure(state='disabled' if self.armed else 'readonly')

    def current_p9(self):
        return int(self.p9.get()) if self.p9.get() in ('0','2') else 2

    def toggle_connection(self):
        if self.link:
            self.disconnect();return
        try:
            self.link = Link(self.port_name.get())
            self.last_status = None;self.last_status_at=0
            self.link.send(Command(p9=self.current_p9()))
            self.connect_button.configure(text='Отключить')
            self.arm_button.configure(state='normal')
            self.notice.set('Подключено. Дождитесь radio=1 и подтверждений ACK; затем разрешите управление.')
        except (OSError,serial.SerialException) as e:
            self.disconnect();messagebox.showerror('COM-порт',str(e))

    def center(self):
        for slider in (self.forward, self.turn):
            state = slider.cget('state')
            slider.configure(state='normal');slider.set(0);slider.configure(state=state)
        self.draw()

    def stop(self, message='Управление снято. Передаётся нейтральная команда.'):
        self.armed=False;self.drive_ready=False;self.center();self.blade=0
        self.cruise.set(False);self.light.set(False);self.set_controls(False)
        self.notice.set(message)
        if self.link:
            try:self.link.send(Command(p9=self.current_p9()))
            except (OSError,serial.SerialException):pass
        self.record('HOST STOP '+message)

    def disconnect(self):
        self.stop('COM-порт отключён.')
        if self.link:
            try:
                self.link.send(Command(p9=self.current_p9()))
            except (OSError,serial.SerialException):pass
            finally:
                try:self.link.close()
                except (OSError,serial.SerialException):pass
            self.link=None
        self.arm_button.configure(state='disabled');self.connect_button.configure(text='Подключить')

    def arm(self):
        s=self.last_status
        if not self.link or not s or time.monotonic()-self.last_status_at>0.6:
            self.notice.set('Нет свежего статуса STM32. Проверьте прошивку и COM-порт.');return
        if s['session']!=self.link.session or not s['radio'] or s['link_ms']>=300:
            self.notice.set('Для разрешения нужна настроенная радиосвязь: radio=1 и свежий ACK или ответ.');return
        self.center();self.blade=0;self.cruise.set(False)
        self.armed=True;self.drive_ready=False;self.arm_time=time.monotonic()
        self.set_controls(False)
        try:self.arm_sequence=self.link.send(Command(armed=True,p9=self.current_p9()))
        except (OSError,serial.SerialException) as e:
            self.stop(str(e));return
        self.notice.set('Ожидание подтверждения разрешения от STM32…')

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
        if self.armed and self.root.focus_displayof() is None:
            self.stop('Окно потеряло фокус: управление снято.')

    def receive(self):
        for line in self.link.read_lines():
            self.record(line)
            s=parse_status(line)
            if s is None:continue
            self.last_status=s;self.last_status_at=time.monotonic()
            fault=FAULTS.get(s['fault'],str(s['fault']))
            self.status.set(f"STM32: radio={s['radio']}  armed={s['armed']}  причина: {fault}\n"
                            f"ACK={s['ack']}  неудач TX={s['fail']}  ответов BB={s['rx']}  возраст связи={s['link_ms']} мс\n"
                            f"Принято: V={s['f']} R={s['t']} нож={s.get('blade')} круиз={s.get('cruise')} свет={s.get('light')}\n"
                            f"Последний ответ: {s.get('reply','-')}")
            if not self.armed:continue
            if s['session']!=self.link.session:
                self.stop('STM32 перезапустилась / изменилась сессия. Требуется повторное разрешение.');continue
            if s['armed']:
                self.drive_ready=True;self.set_controls(True)
                self.notice.set('УПРАВЛЕНИЕ РАЗРЕШЕНО. Пробел или красная кнопка — остановка.')
            elif self.drive_ready or ((s['seq']-self.arm_sequence)&0xFFFFFFFF) < 0x80000000:
                self.stop('STM32 сняла/отклонила разрешение: '+fault)

    def tick(self):
        if self.closing:return
        now=time.monotonic();gap=now-self.last_tick;self.last_tick=now
        if self.armed and gap>0.20:self.stop('Пауза Python более 200 мс: требуется повторное разрешение.')
        if self.link:
            try:
                self.receive()
                if self.armed and (now-self.last_status_at>0.7 or
                                  (not self.drive_ready and now-self.arm_time>0.7)):
                    self.stop('Нет подтверждения STM32: управление снято.')
                cmd=Command(p9=self.current_p9())
                if self.armed:
                    cmd=Command(forward=self.forward.get() if self.drive_ready else 0,
                                turn=self.turn.get() if self.drive_ready else 0,
                                blade=self.blade if self.drive_ready else 0,
                                cruise=self.cruise.get() if self.drive_ready else False,
                                light=self.light.get() if self.drive_ready else False,
                                armed=True,p9=self.current_p9())
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
