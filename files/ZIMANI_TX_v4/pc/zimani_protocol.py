"""Host binary v2 for ZimAni STM32 TX v4. No hardware side effects.
Default Command asserts STOP. Releasing requires an explicit reset_stop frame.
"""
from dataclasses import dataclass
import struct

@dataclass(frozen=True)
class Command:
    forward: int = 0
    turn: int = 0
    blade: int = 0
    cruise: bool = False
    light: bool = False
    armed: bool = False
    engine: int = 2
    engine_adjust: int = 0
    estop: bool = True
    reset_stop: bool = False

def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value

def encode(session: int, sequence: int, command: Command) -> bytes:
    if not 0 < session <= 0xFFFFFFFF or not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError('Invalid session/sequence')
    if not -255 <= command.forward <= 255 or not -255 <= command.turn <= 255:
        raise ValueError('Axis must be in -255..255')
    if (command.blade not in (0, 1, 2) or command.engine not in (0, 2)
            or command.engine_adjust not in (0, 1, 2)):
        raise ValueError('Invalid blade, engine or engine_adjust value')
    for name in ('armed','cruise','light','estop','reset_stop'):
        if getattr(command,name) not in (False,True):
            raise ValueError('Flags must be boolean')
    flags = (int(command.armed) | (int(command.cruise) << 1) | (int(command.light) << 2)
             | (int(command.estop) << 3) | (int(command.reset_stop) << 4))
    data = struct.pack('<2sBBIIhhBBBB', b'\xC3\x3C', 2, 1, session, sequence,
                       command.forward, command.turn, command.blade, flags, command.engine, command.engine_adjust)
    return data + struct.pack('<H', crc16(data))

def parse_status(line: str):
    if not line.startswith('S '):
        return None
    try:
        items = dict(item.split('=', 1) for item in line.split()[1:])
        out = {k: (v if k == 'reply' else int(v)) for k, v in items.items()}
        required = {'v','session','seq','armed','radio','fault','host_ms','link_ms','ack','fail','rx','f','t',
                    'engine','adjust','estop','can_arm','stop_rx','rx_ms'}
        if out['v'] != 4 or not required.issubset(out):
            return None
        return out
    except (KeyError, ValueError):
        return None
