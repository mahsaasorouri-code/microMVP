import time

# 1) 导入你的 comm.py 里的类（把类名按你实际的改）
from comm import UsbAPComm


class DummyApp:
    def __init__(self, port="COM7", baudrate=115200, payload_level=2, framed=True):
        # 2) 这里创建 comm 对象
        self.comm = UsbAPComm(
            port=port,
            baudrate=baudrate,
            payload_level=payload_level,
            framed=framed,
        )


def send_for(app: DummyApp, car_id: int, left: float, right: float, duration_s: float, hz: float = 30.0):
    """
    注意：这里 left/right 是 float，和 GUI 一样（GUI 传的是 lSpeed/rSpeed * simSpeed）
    你 comm.SendId 内部会量化成 byte。
    """
    dt = 1.0 / hz
    t0 = time.time()
    while time.time() - t0 < duration_s:
        app.comm.SendId([car_id], [left], [right])
        time.sleep(dt)


def stop(app: DummyApp, car_id: int):
    app.comm.SendId([car_id], [0.0], [0.0])
    time.sleep(0.2)


def main():
    CAR_ID = 1

    # 速度（和 GUI 一样用 float，通常范围 0~1 左右；如果你的 SendId 支持 0~1 映射到 0~127）
    # 如果你发现太慢/太快，就把 V/TURN 改一下
    V = 0.55
    TURN = 0.55
    DUR = 2.0

    app = DummyApp(port="COM7", baudrate=115200, payload_level=2, framed=True)

    print("== Forward ==")
    send_for(app, CAR_ID, left=+V, right=+V, duration_s=DUR)
    stop(app, CAR_ID)

    print("== Backward ==")
    send_for(app, CAR_ID, left=-V, right=-V, duration_s=DUR)
    stop(app, CAR_ID)

    print("== Turn Left (in place) ==")
    send_for(app, CAR_ID, left=-TURN, right=+TURN, duration_s=DUR)
    stop(app, CAR_ID)

    print("== Turn Right (in place) ==")
    send_for(app, CAR_ID, left=+TURN, right=-TURN, duration_s=DUR)
    stop(app, CAR_ID)

    print("Done.")


if __name__ == "__main__":
    main()
