import time
import serial
import msvcrt

#发送的信息[ L0 ][ L1 ][ R1 ][ L2 ][ R2 ][ L3 ][ R3 ]
# -----------------------
# Config
# -----------------------
PORT = "COM7"
BAUD = 115200

CAR_ID = 1
CARS_PER_LEVEL = 10
PAYLOAD_LEN = 32

HZ = 30.0
DT = 1.0 / HZ

FWD = 70
TURN = 70

HDR0 = 0xAA
HDR1 = 0x55

# -----------------------
# Helpers
# -----------------------
def car_level_index(car_id: int) -> tuple[int, int]:
    level = (car_id - 1) // CARS_PER_LEVEL
    idx = car_id - level * CARS_PER_LEVEL  # 1..10
    return level, idx

def encode_byte(thrust: int) -> int:
    thrust = max(-127, min(127, int(thrust)))
    mag = abs(thrust) & 0x7F
    return mag | 0x80 if thrust < 0 else mag

def checksum_xor(level: int, ln: int, payload: bytes) -> int:
    c = 0
    c ^= (level & 0xFF)
    c ^= (ln & 0xFF)
    for b in payload:
        c ^= b
    return c & 0xFF

def build_payload32(car_id: int, left: int, right: int) -> tuple[int, bytes]:
    level, idx = car_level_index(car_id)
    p = bytearray(PAYLOAD_LEN)

    # slot mapping (same as your comm.py / teacher)
    left_pos = 3 * idx - 1
    right_pos = 3 * idx

    p[left_pos] = encode_byte(left)
    p[right_pos] = encode_byte(right)

    return level, bytes(p)

def build_frame(level: int, payload32: bytes) -> bytes:
    ln = PAYLOAD_LEN
    cks = checksum_xor(level, ln, payload32)
    return bytes([HDR0, HDR1, level & 0xFF, ln & 0xFF]) + payload32 + bytes([cks])

def print_help():
    print("Controls (toggle mode):")
    print("  W: forward   S: backward")
    print("  A: turn left D: turn right (in-place)")
    print("  Space: stop (one-shot)")
    print("  +/- : increase/decrease FWD")
    print("  [/]: increase/decrease TURN")
    print("  Q: quit")
    print()

def command_from_keys(keys: set[int], fwd: int, turn: int) -> tuple[int, int]:
    if ord(' ') in keys:
        return 0, 0

    left = 0
    right = 0

    if ord('W') in keys:
        left += fwd
        right += fwd
    if ord('S') in keys:
        left -= fwd
        right -= fwd

    if ord('A') in keys:
        left -= turn
        right += turn
    if ord('D') in keys:
        left += turn
        right -= turn

    left = max(-127, min(127, left))
    right = max(-127, min(127, right))
    return left, right

# -----------------------
# Main
# -----------------------
def main():
    global FWD, TURN

    level, idx = car_level_index(CAR_ID)
    print(f"Opening {PORT} @ {BAUD} ...")
    ser = serial.Serial(PORT, BAUD, timeout=0, write_timeout=0.2)
    time.sleep(0.2)

    print_help()
    print(f"CAR_ID={CAR_ID}  level={level} idx={idx}  send={HZ:.1f}Hz")
    print("NOTE: This sends framed bytes: AA 55 level 32 payload32 xor_cksum")
    print()

    keys: set[int] = set()
    last_send = time.time()
    last_ui = 0.0

    try:
        while True:
            # keyboard (toggle)
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if not ch:
                    continue
                if ch in (b'\x00', b'\xe0'):
                    _ = msvcrt.getch()
                    continue

                c = ch.decode(errors="ignore").upper()
                if c == 'Q':
                    return

                if c == '+':
                    FWD = min(127, FWD + 5)
                elif c == '-':
                    FWD = max(0, FWD - 5)
                elif c == '[':
                    TURN = max(0, TURN - 5)
                elif c == ']':
                    TURN = min(127, TURN + 5)
                elif c == ' ':
                    keys = {ord(' ')}  # one-shot stop
                elif c in ('W', 'A', 'S', 'D'):
                    code = ord(c)
                    if code in keys:
                        keys.remove(code)
                    else:
                        keys.add(code)

            one_shot_stop = (ord(' ') in keys)

            left, right = command_from_keys(keys, FWD, TURN)

            now = time.time()
            if now - last_send >= DT:
                last_send = now

                level, payload32 = build_payload32(CAR_ID, left, right)
                frame = build_frame(level, payload32)
                ser.write(frame)

                if one_shot_stop:
                    keys.clear()

            if now - last_ui > 0.25:
                last_ui = now
                show = ''.join(chr(k) for k in sorted(keys) if k != ord(' '))
                print(f"\rFWD={FWD:3d} TURN={TURN:3d}  L={left:4d} R={right:4d}  keys={show:<4}   ", end="")

            time.sleep(0.001)

    finally:
        try:
            level, payload32 = build_payload32(CAR_ID, 0, 0)
            ser.write(build_frame(level, payload32))
        except Exception:
            pass
        ser.close()
        print("\nClosed.")

if __name__ == "__main__":
    main()
