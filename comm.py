# comm.py
# -*- coding: utf-8 -*-

from __future__ import annotations

import time
from typing import List, Sequence, Optional, Dict

import serial  # pip install pyserial


def _crc8_xor(data: bytes) -> int:
    """Cheap checksum: XOR of all bytes."""
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


class UsbAPComm:
    """
    PC -> USB Serial -> Xiao_AP

    Public API:
      - SendId(idlist, lefts, rights): build CrazyRadio-style 32B payload(s) and send
      - Flush(): clear payload(s) and send (stop/clear buffer)

    Payload packing is modeled after CrazyRadioSendId:
      carLevel = (carID-1)//10
      carIndex = carID - carLevel*10   # 1..10

      payload[3*carIndex]     = right magnitude (0..127) with sign in MSB
      payload[3*carIndex - 1] = left  magnitude (0..127) with sign in MSB

    Each level payload length is 32 bytes.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        payload_len: int = 32,
        payload_level: int = 2,
        framed: bool = True,
        write_timeout_s: float = 0.05,
    ) -> None:
        """
        payload_level:
          how many levels exist (e.g., 2 => supports carID 1..20).
          Set it big enough for your fleet.

        framed:
          If True, PC->AP uses AA55 + level + len + payload + checksum
          If False, send ONLY raw 32B payload (NOT recommended unless AP reads fixed blocks perfectly).
        """
        if payload_len != 32:
            raise ValueError("This implementation assumes 32-byte payloads (payload_len must be 32).")
        if payload_level <= 0:
            raise ValueError("payload_level must be >= 1")

        self.port = port
        self.baudrate = baudrate
        self.payload_len = payload_len
        self.payload_level = payload_level
        self.framed = framed

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0,  # non-blocking read (we only write)
            write_timeout=write_timeout_s,
        )

        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

        # internal cmd table cache (level -> bytearray[32])
        self._cmd_table: List[bytearray] = [self._empty_payload() for _ in range(self.payload_level)]

    # ---------------------------
    # Public API (exactly 2 funcs)
    # ---------------------------

    def SendId(self, idlist: Sequence[int], lefts: Sequence[float], rights: Sequence[float]) -> None:

        """
        Build 32B payload(s) following CrazyRadioSendId semantics and send to AP over USB.
        """
        if not (len(idlist) == len(lefts) == len(rights)):
            raise ValueError(
                f"Length mismatch: len(idlist)={len(idlist)} len(lefts)={len(lefts)} len(rights)={len(rights)}"
            )

        # Start from existing table (or clear first—your call). Here we overwrite only specified cars.
        # If you prefer "each SendId sends a full snapshot", uncomment the next line:
        # self._cmd_table = [self._empty_payload() for _ in range(self.payload_level)]

        for idx, car_id in enumerate(idlist):
            car_id_i = int(car_id)
            if car_id_i <= 0:
                continue

            level = (car_id_i - 1) // 10  # 0-based
            if level < 0 or level >= self.payload_level:
                # silently ignore out-of-range IDs (or raise if you prefer)
                continue

            car_index = car_id_i - level * 10  # 1..10

            # Match CrazyRadio indices:
            right_pos = 3 * car_index
            left_pos = 3 * car_index - 1

            if right_pos >= self.payload_len or left_pos >= self.payload_len or left_pos < 0:
                # should not happen for 32B + car_index in 1..10
                continue

            r = float(rights[idx])
            l = float(lefts[idx])

            rb = int(abs(r) * 127) & 0x7F
            lb = int(abs(l) * 127) & 0x7F

            if r < 0:
                rb |= 0x80
            if l < 0:
                lb |= 0x80

                
            # if car_id_i in (1,4,5,6,7):
            #     print("SendId write", car_id_i, "level", level, "car_index", car_index,
            #         "L", l, "R", r, "lb", lb, "rb", rb, "left_pos", left_pos, "right_pos", right_pos)
            payload = self._cmd_table[level]
            payload[right_pos] = rb
            payload[left_pos] = lb

        # Send all levels (like CrazyRadioSendId does)
        for level in range(self.payload_level):
            self._send_level(level, bytes(self._cmd_table[level]))

    def Flush(self) -> None:
        """
        Similar to CrazyRadioFlush7:
          clear cmd table, then send all payload levels.
        """
        self._cmd_table = [self._empty_payload() for _ in range(self.payload_level)]
        for level in range(self.payload_level):
            self._send_level(level, bytes(self._cmd_table[level]))

    # ---------------------------
    # Internals
    # ---------------------------

    def _empty_payload(self) -> bytearray:
        return bytearray([0] * self.payload_len)

    def _send_level(self, level: int, payload32: bytes) -> None:
        if len(payload32) != 32:
            raise ValueError("payload must be exactly 32 bytes")

        if not self.framed:
            # Raw-only: send 32 bytes (AP must read exactly 32 each time, per level, in order)
            self._ser.write(payload32)
            return
            # AA 55 | level(1) | len=32(1) | payload32(32) | chk(1, XOR)
        lvl = level & 0xFF
        ln = 32
        body = bytes([lvl, ln]) + payload32
        chk = _crc8_xor(body)
        frame = bytes([0xAA, 0x55]) + body + bytes([chk])
        self._ser.write(frame)

    def close(self) -> None:
        try:
            self.Flush()
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass
