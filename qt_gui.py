import sys, os, math, threading, time, argparse
from queue import Queue
from os import listdir
from os.path import isfile, join
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QPen, QFont, QPixmap, QTransform
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QSlider, QComboBox,
    QLabel, QVBoxLayout, QHBoxLayout, QMessageBox
)
from PyQt6.QtCore import QSize
from random import random
from munkres import Munkres

import utils
import positionZMQSub
import DDR
import importlib.util
import zmq
import subprocess
import atexit


VISION_PUB_PORT = 5556  
VISION_SCRIPT = os.path.join(os.path.dirname(__file__), "vision_pub.py")

_vision_proc = None

def _is_port_bound(port: int) -> bool:
    """Return True if tcp://0.0.0.0:port cannot be bound (likely already in use)."""
    ctx = zmq.Context.instance()
    s = ctx.socket(zmq.PUB)
    try:
        s.bind(f"tcp://0.0.0.0:{port}")
        s.unbind(f"tcp://0.0.0.0:{port}")
        return False
    except zmq.ZMQError:
        return True
    finally:
        s.close(0)

def start_vision_pub_if_needed() -> subprocess.Popen | None:
    global _vision_proc

    if _is_port_bound(VISION_PUB_PORT):
        print(f"[qt_gui] Port {VISION_PUB_PORT} already in use; skip starting vision_pub.")
        return None

    python_exe = sys.executable 
    cmd = [python_exe, VISION_SCRIPT]
    print(f"[qt_gui] Starting vision_pub: {' '.join(cmd)}")


    creationflags = 0
    if os.name == "nt":
        creationflags = subprocess.CREATE_NO_WINDOW

    _vision_proc = subprocess.Popen(
        cmd,
        cwd=os.path.dirname(__file__),
        stdout=subprocess.DEVNULL,  
        stderr=subprocess.DEVNULL,
        creationflags=creationflags
    )

    def _cleanup():
        global _vision_proc
        if _vision_proc is not None and _vision_proc.poll() is None:
            print("[qt_gui] Terminating vision_pub...")
            _vision_proc.terminate()
            try:
                _vision_proc.wait   (timeout=2)
            except Exception:
                _vision_proc.kill()
        _vision_proc = None

    atexit.register(_cleanup)

    time.sleep(0.2)
    return _vision_proc

def _load_source(name, path):
    spec = importlib.util.spec_from_file_location("dynamic_mod", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

lock = threading.Lock()


class Canvas(QWidget):
    """Qt painter widget: draw cars + paths using QPainter."""
    def __init__(self, app_ref):
        super().__init__()
        self.app_ref = app_ref
        self.setFixedSize(utils.painter_width, utils.painter_height)

        self.font = QFont("Arial", max(10, int(utils.wheelBase)))


        raw = QPixmap(os.path.join("images", "carImage.png"))

        w = int(utils.wheelBase * 2.0 / (9.0 / 8.0))
        h = int(utils.wheelBase * 2.0)

        self.car_pix = raw.scaled(w, h, Qt.AspectRatioMode.IgnoreAspectRatio,
                                Qt.TransformationMode.SmoothTransformation)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        # background
        painter.fillRect(self.rect(), Qt.GlobalColor.white)

        # take snapshot
        with lock:
            cars = list(self.app_ref.cars.values())
            bound = self.app_ref.bound

        # boundary
        pen = QPen(Qt.GlobalColor.gray)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawRect(int(bound.l), int(bound.u), int(bound.width), int(bound.height))

        # draw paths + goals
        for idx, car in enumerate(cars):
            path = list(car.path)
            if len(path) >= 2:
                color = utils.RGB_PATH_COLORS[idx % len(utils.RGB_PATH_COLORS)]
                pen = QPen(Qt.GlobalColor.black)
                pen.setWidth(3)
                pen.setColor(Qt.GlobalColor.black)
                # Qt needs QColor; easiest: use setPen via QColor from rgb
                from PyQt6.QtGui import QColor
                pen.setColor(QColor(*color))
                painter.setPen(pen)

                for i in range(len(path) - 1):
                    painter.drawLine(int(path[i][0]), int(path[i][1]), int(path[i+1][0]), int(path[i+1][1]))
            
                # goal marker
                gx, gy = path[-1]
                painter.drawEllipse(int(gx) - 4, int(gy) - 4, 8, 8)
                painter.setFont(self.font)
                painter.drawText(int(gx) + 6, int(gy) + 6, f"Goal{car.ID}")

        # draw cars
        for idx, car in enumerate(cars):
            x, y, th = car.x, car.y, car.theta
            color = utils.RGB_PATH_COLORS[idx % len(utils.RGB_PATH_COLORS)]

            # rotate pixmap: your pygame used -180*theta/pi - 90
            angle_deg = 180.0 * th / math.pi + 90.0
            tr = QTransform().rotate(angle_deg)
            rotated = self.car_pix.transformed(tr, Qt.TransformationMode.SmoothTransformation)

            # draw centered at (x,y)
            cx = int(x - rotated.width() / 2)
            cy = int(y - rotated.height() / 2)
            painter.drawPixmap(cx, cy, rotated)

            # label id
            from PyQt6.QtGui import QColor
            painter.setPen(QColor(*color))
            painter.setFont(self.font)
            painter.drawText(int(x), int(y), str(car.ID))
            
            arrow_len = 40
            hx = x + arrow_len * math.cos(th)
            hy = y + arrow_len * math.sin(th)
            painter.drawLine(int(x), int(y), int(hx), int(hy))
            # =================================

        painter.end()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("microMVP (PyQt)")

        self.sim = False
        self.vMax = 1.0
        self.simSpeed = utils.simSpeed
        self.runCar = False
        self.syn = False
        self.testflag = True

        self.bound = utils.Boundary()

        # cars dict
        self.cars = {}

        self._setup_argv_and_comm()
        self._setup_cars()

        # UI
        root = QWidget()
        self.setCentralWidget(root)

        left = QVBoxLayout()
        btn_run = QPushButton("Run")
        btn_stop = QPushButton("Stop")
        btn_clear = QPushButton("Clear")
        btn_about = QPushButton("About")

        btn_run.clicked.connect(self.B_run)
        btn_stop.clicked.connect(self.B_stop)
        btn_clear.clicked.connect(self.B_clear)
        btn_about.clicked.connect(self.B_about)

        left.addWidget(QLabel("Car Control:"))
        left.addWidget(btn_run)
        left.addWidget(btn_stop)
        left.addWidget(btn_clear)

        left.addWidget(QLabel("Speed:"))
        self.sli_v = QSlider(Qt.Orientation.Horizontal)
        self.sli_v.setRange(0, 100)
        self.sli_v.setValue(100)
        left.addWidget(self.sli_v)

        left.addSpacing(12)
        left.addWidget(QLabel("Draw Path Car:"))
        self.sel_car = QComboBox()
        for carID, tag in utils.carInfo:
            self.sel_car.addItem(f"#{carID}, Tag{tag}", carID)
        left.addWidget(self.sel_car)

        # -------- Pattern UI --------
        left.addSpacing(12)
        left.addWidget(QLabel("Patterns:"))
        self.sel_ptn = QComboBox()
        ptn_files = [f for f in listdir("patterns/") if isfile(join("patterns/", f))]

        ALLOWED_PATTERNS = {
            "circle1.py",
            "circle2.py",
            "figure8_2.py",
        }

        for f in ptn_files:
            if f in ALLOWED_PATTERNS:
                self.sel_ptn.addItem(f.split(".")[0], f)

        left.addWidget(self.sel_ptn)

        btn_pattern = QPushButton("Get Pattern")
        btn_pattern.clicked.connect(self.B_pattern)
        left.addWidget(btn_pattern)

        # -------- Algorithm UI --------
        left.addSpacing(12)
        left.addWidget(QLabel("Path Planning:"))
        self.sel_alg = QComboBox()
        alg_files = [f for f in listdir("algorithms/") if isfile(join("algorithms/", f))]
        ALLOWED_ALGS = {"mrpp_b.py", "rvo2.py"}
        for f in alg_files:
            if f in ALLOWED_ALGS:
                self.sel_alg.addItem(f.split(".")[0], f)

        left.addWidget(self.sel_alg)

        btn_plan = QPushButton("Run ALG")
        btn_plan.clicked.connect(self.B_plan)
        left.addWidget(btn_plan)

        # -------- Test button --------
        left.addSpacing(12)
        btn_test = QPushButton("Test")
        btn_test.clicked.connect(self.B_test)
        left.addWidget(btn_test)


        left.addStretch(1)
        left.addWidget(btn_about)

        self.canvas = Canvas(self)

        main = QHBoxLayout()
        main.addLayout(left)
        main.addWidget(self.canvas)

        root.setLayout(main)

        # timer refresh UI (30 FPS)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.canvas.update)
        self.timer.start(33)

        # mouse path
        self.canvas.setMouseTracking(True)
        self.canvas.mousePressEvent = self._mouse_press
        self.canvas.mouseReleaseEvent = self._mouse_release
        self.canvas.mouseMoveEvent = self._mouse_move
        self._drawing = False

        # threads
        self._start_threads()

    def _setup_argv_and_comm(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-s", dest="sim", action="store_true", default=False)
        args, _ = parser.parse_known_args()
        self.sim = bool(os.environ.get("sim", args.sim))

        if not self.sim:
            self.simSpeed = 0.99
            from comm import UsbAPComm
            self.comm = UsbAPComm(port="COM7", baudrate=115200, payload_level=2, framed=True)
            self.comm.Flush()

    def _setup_cars(self):
        # detect cars
        if not self.sim:
            positionZMQSub._pull_zmq_data_once()
            positionZMQSub._initialize_zmq()
        else:
            utils.carInfo = [(i, i) for i in range(1, 7)]  # simulate 6 cars with tag = ID

        with lock:
            self.cars.clear()
            for carID, tag in utils.carInfo:
                self.cars[carID] = utils.UnitCar(tag=tag, ID=carID)

        # setup wheelBase if real
        if not self.sim:
            self._setup_wheelbase_from_tags()
        
        if self.sim:
            self.bound = utils.Boundary()
            self.GetRandomArrangement()
        else:
            positionZMQSub._initialize_zmq()
            self.bound = utils.Boundary()

        self.bound = utils.Boundary()

    def _setup_wheelbase_from_tags(self):
        data = positionZMQSub._get_all_car_position_data()
        read = 0
        wb_sum = 0.0
        for carID, tag in utils.carInfo:
            if data.get(tag, "") == "":
                continue
            x0, y0, x1, y1, x2, y2, x3, y3 = data[tag].split()
            tagSize = math.hypot(float(x0)-float(x1), float(y0)-float(y1))
            wb_sum += tagSize / utils.tagRatio
            read += 1
        if read > 0:
            utils.wheelBase = wb_sum / read

    # -------- UI callbacks --------
    def B_run(self):
        with lock:
            self.runCar = True

    def B_stop(self):
        with lock:
            self.runCar = False

    def B_clear(self):
        with lock:
            for car in self.cars.values():
                car.path = []
            if (not self.sim) and hasattr(self, "comm"):
                self.comm.Flush()

    def B_pattern(self):
        if self.sel_ptn.currentData() is None:
            return

        # stop first
        self.B_stop()

        # load pattern module
        fname = self.sel_ptn.currentData()
        mod = _load_source("", os.path.join("patterns", fname))

        # pattern gives desired paths
        paths = mod.GetPath(len(self.cars.keys()), self.bound)

        # current locations
        with lock:
            locs = [(c.x, c.y) for c in self.cars.values()]

        # assign cars to paths (munkres)
        from munkres import Munkres
        paths2 = self.Shuffle(locs, paths)
        paths2 = self.Refinement(paths2)

        # connect to starts using rvo2 (same as your old code)
        rvo2 = _load_source("", os.path.join("algorithms", "rvo2.py"))
        paths1 = rvo2.GetPath(locs, [p[0] for p in paths2], utils.wheelBase, self.bound)
        paths1 = self.Refinement(paths1)

        # small middle connection
        pathsm = [[paths1[i][-1], paths2[i][0]] for i in range(len(paths2))]
        pathsm = self.Refinement(pathsm)

        # merge
        merged = []
        for i in range(len(paths2)):
            merged.append(paths1[i] + pathsm[i] + paths2[i])

        # commit
        with lock:
            self.syn = True
            # Note: `self.cars` is keyed by carID; dict.values() order is not guaranteed
            # Therefore write paths using the stable order from `utils.carInfo`.
            for idx, (carID, _) in enumerate(utils.carInfo):
                self.cars[carID].path = merged[idx]


    def B_about(self):
        QMessageBox.information(self, "About", "microMVP (PyQt6 UI)")

    # -------- Mouse drawing (screen coords = painter coords here) --------
    def _mouse_press(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drawing = True

    def _mouse_release(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drawing = False

    def _mouse_move(self, e):
        if not self._drawing:
            return
        carID = self.sel_car.currentData()
        if carID is None:
            return

        x, y = int(e.position().x()), int(e.position().y())

        with lock:
            path = self.cars[carID].path
            if len(path) == 0 or math.hypot(x - path[-1][0], y - path[-1][1]) >= 10:
                path.append((x, y))
                self.syn = False


    # -------- Threads (backend) --------
    def _start_threads(self):
        self.running = True  # add running flag
        t_loc = threading.Thread(target=self.GetLocation, daemon=True)
        t_follow = threading.Thread(target=self.Follow, daemon=True)
        t_send = threading.Thread(target=self.SendSpeed, daemon=True)
        t_loc.start()
        t_follow.start()
        t_send.start()

    def Follow(self):
        while True:
            # 1) Pause Follow during testing
            with lock:
                pass_test = (not self.testflag)

            if pass_test:
                time.sleep(0.01)
                continue

            # 2) Read locs/paths in the stable order of utils.carInfo
            with lock:
                locs = [
                    (self.cars[carID].x, self.cars[carID].y, self.cars[carID].theta)
                    for carID, _ in utils.carInfo
                ]
                paths = [
                    list(self.cars[carID].path)
                    for carID, _ in utils.carInfo
                ]
                vM = self.sli_v.value() / 100.0 * self.vMax
                syn = self.syn


            speeds = [(0.0, 0.0) for _ in locs]
            for i, (x, y, th) in enumerate(locs):
                speeds[i] = DDR.Calculate(x, y, th, paths[i], vM, utils.wheelBase)


            if syn:
                self.Synchronize(speeds, paths)

            with lock:
                for (carID, _), sp, path in zip(utils.carInfo, speeds, paths):
                    self.cars[carID].lSpeed = sp[0]
                    self.cars[carID].rSpeed = sp[1]
                    self.cars[carID].path = path

            time.sleep(0.01)

    
    def Synchronize(self, speeds, paths):
        length = max((len(p) for p in paths), default=0)
        if length == 0:
            return
        for i, p in enumerate(paths):
            gap = length - len(p)                 # difference in path lengths
            diff = float(12 - gap) / 12.0         # map to roughly [0,1]
            if diff < 0.0:
                diff = 0.0

            # set a minimum scale to avoid speeds going to (0,0)
            # when diff == 0.
            scale = max(0.2, math.sqrt(diff))

            speeds[i] = (speeds[i][0] * scale, speeds[i][1] * scale)

    def GetLocation(self):
        if self.sim:
            while True:
                with lock:
                    run = self.runCar
                    locs = [(self.cars[carID].x, self.cars[carID].y, self.cars[carID].theta) for carID, _ in utils.carInfo]
                    speeds = [(self.cars[carID].lSpeed * self.simSpeed,
                            self.cars[carID].rSpeed * self.simSpeed) for carID, _ in utils.carInfo]

                if run:
                    new_locs = []
                    for (x, y, th), (vL, vR) in zip(locs, speeds):
                        nx, ny, nth = DDR.Simulate(x, y, th, vL, vR, utils.wheelBase)
                        new_locs.append((nx, ny, nth))

                    with lock:
                        for (carID, _), (nx, ny, nth) in zip(utils.carInfo, new_locs):
                            self.cars[carID].x = nx
                            self.cars[carID].y = ny
                            self.cars[carID].theta = nth
                else:
                    time.sleep(0.001)

                time.sleep(0.001)
        else:
            while True:
                data = positionZMQSub._get_all_car_position_data()
                with lock:
                    for carID, tag in utils.carInfo:
                        s = data.get(tag, "")
                        if not s:
                            continue
                        x0,y0,x1,y1,x2,y2,x3,y3 = map(float, s.split())
                        x = (x0 + x1 + x2 + x3) / 4.0
                        y = (y0 + y1 + y2 + y3) / 4.0
                        frontMid_x = (x0 + x1) / 2.0
                        frontMid_y = (y0 + y1) / 2.0
                        rareMid_x  = (x2 + x3) / 2.0
                        rareMid_y  = (y2 + y3) / 2.0
                        theta = DDR.calculateATan(frontMid_x - rareMid_x, frontMid_y - rareMid_y)
                        self.cars[carID].x = x
                        self.cars[carID].y = y
                        self.cars[carID].theta = theta
                time.sleep(0.005)

    def SendSpeed(self):
        if self.sim:
            return
        while True:
            with lock:
                run = self.runCar
                idlist = [carID for carID, _ in utils.carInfo]
                lefts  = [self.cars[carID].lSpeed * self.simSpeed for carID, _ in utils.carInfo]
                rights = [self.cars[carID].rSpeed * self.simSpeed for carID, _ in utils.carInfo]
                
                # # DEBUG: check selected cars
                # watch = {1,4,5,6,7}
                # for carID in idlist:
                #     if carID in watch:
                #         L = self.cars[carID].lSpeed * self.simSpeed
                #         R = self.cars[carID].rSpeed * self.simSpeed
                #         print("[PRE SEND]", carID, "L", L, "R", R)
            if run:
                self.comm.SendId(idlist, lefts, rights)
            else:
                self.comm.Flush()

            time.sleep(0.02)

    def closeEvent(self, event):
        self.running = False 
        time.sleep(0.1)      
        try:
            if (not self.sim) and hasattr(self, "comm"):
                self.comm.Flush()
                self.comm.close()
        except:
            pass
        event.accept()

    def B_test(self):
        if self.sim:
            self.B_clear()
            print("Testing is not for simulation mode")
            return

        # pause Follow
        with lock:
            self.testflag = False
            self.runCar = False

        time.sleep(0.1)

        # snapshot current loc
        with lock:
            current = {carID: (self.cars[carID].x, self.cars[carID].y, self.cars[carID].theta)
                    for carID, _ in utils.carInfo}

        allGood = True

        # helper: run one spin direction
        def run_spin(l_sign, r_sign, seconds=1.0):
            nonlocal allGood
            # clear paths + set speeds
            with lock:
                for carID, _ in utils.carInfo:
                    self.cars[carID].path = []
                    self.cars[carID].lSpeed = l_sign * (1.0 / self.simSpeed)
                    self.cars[carID].rSpeed = r_sign * (1.0 / self.simSpeed)
                self.runCar = True

            # monitor motion for N steps
            steps = int(seconds / 0.1)
            dist_max = {carID: 0.0 for carID, _ in utils.carInfo}
            still_cnt = {carID: 0 for carID, _ in utils.carInfo}

            for k in range(steps):
                time.sleep(0.1)
                with lock:
                    for carID, _ in utils.carInfo:
                        px, py, pth = current[carID]
                        cx, cy, cth = self.cars[carID].x, self.cars[carID].y, self.cars[carID].theta
                        d = math.hypot(cx - px, cy - py)
                        dist_max[carID] = max(dist_max[carID], d)

                        if abs(pth - cth) <= 0.05:
                            still_cnt[carID] += 1

            # judge
            for carID, _ in utils.carInfo:
                if dist_max[carID] > 0.5 * utils.wheelBase:
                    allGood = False
                    print(carID, "wheel has problem!")
                if still_cnt[carID] == steps:
                    allGood = False
                    print(carID, "not moving!")

            # stop
            with lock:
                for carID, _ in utils.carInfo:
                    self.cars[carID].lSpeed = 0.0
                    self.cars[carID].rSpeed = 0.0
                self.runCar = False

            time.sleep(0.5)

            # refresh baseline
            with lock:
                for carID, _ in utils.carInfo:
                    current[carID] = (self.cars[carID].x, self.cars[carID].y, self.cars[carID].theta)

        # right spin then left spin (matching your old intent)
        self.B_stop()
        run_spin(l_sign=-1.0, r_sign=+1.0, seconds=1.0)
        run_spin(l_sign=+1.0, r_sign=-1.0, seconds=1.0)

        if allGood:
            print("All vehicles work well!")

        # resume Follow
        with lock:
            self.testflag = True

    def B_plan(self):

        #    self.sel_alg.addItem("MRPP", "mrpp_b.py")
        #    self.sel_alg.addItem("RVO2", "rvo2.py")
        alg_file = self.sel_alg.currentData()
        if alg_file is None:
            return
        

        # stop first
        self.B_stop()

        # 1) load algorithm module (safe path join)
        alg_path = os.path.join("algorithms", alg_file)
        if not os.path.exists(alg_path):
            print(f"[B_plan] algorithm file not found: {alg_path!r}")
            return

        mod = _load_source("", alg_path)

        # 2) current locations (stable order: use utils.carInfo, not dict order)
        with lock:
            # utils.carInfo is [(carID, ...), ...]
            car_ids = [carID for (carID, _) in utils.carInfo]
            locs = [(self.cars[carID].x, self.cars[carID].y) for carID in car_ids]

        # 3) goals: for MRPP-style planners, you usually want deterministic count = num cars
        goals = self.GetRandomGoals()  # keep your existing API

        # 4) planner gives paths
        paths2 = mod.GetPath(locs, goals, utils.wheelBase, self.bound)

        # basic sanity check
        if not isinstance(paths2, list) or len(paths2) != len(locs):
            print(f"[B_plan] bad paths from {alg_file}: expected {len(locs)} paths, got {type(paths2)} len={getattr(paths2,'__len__',lambda:None)()}")
            return
        if any((not p) for p in paths2):
            print(f"[B_plan] some paths are empty from {alg_file}")
            return

        # 5) connect to starts using rvo2 (same style as B_pattern)
        rvo2 = _load_source("", os.path.join("algorithms", "rvo2.py"))
        # connect from current locs -> first waypoint of each planned path
        starts = [p[0] for p in paths2]
        paths1 = rvo2.GetPath(locs, starts, utils.wheelBase, self.bound)

        # 6) refinement + merge
        paths1 = self.Refinement(paths1)
        paths2 = self.Refinement(paths2)

        # small middle connection (helps if refinement trims ends)
        pathsm = [[paths1[i][-1], paths2[i][0]] for i in range(len(paths2))]
        pathsm = self.Refinement(pathsm)

        merged = []
        for i in range(len(paths2)):
            merged.append(paths1[i] + pathsm[i] + paths2[i])

        # 7) painter mode (optional)
        # with self.painter.lock2:
        #     if alg_file == "mrpp_b.py":
        #         self.painter.showMode = 1
        #     elif alg_file == "rvo2.py":
        #         self.painter.showMode = 2
        #     else:
        #         # default / unknown algorithm display mode
        #         self.painter.showMode = 0

        # 8) commit (stable order)
        with lock:
            self.syn = True
            for idx, carID in enumerate(car_ids):
                self.cars[carID].path = merged[idx]


    def Shuffle(self, locs, paths):
        #connect current location to the start point of desired path
        matrix = list()
        for index, loc in enumerate(locs):
            matrix.append(list())
            for path in paths:
                matrix[-1].append(math.sqrt(math.pow(loc[0] - path[0][0], 2) + math.pow(loc[1] - path[0][1], 2)))
        m = Munkres()
        indexes = m.compute(matrix)
        newPath = list()
        for row, column in indexes:
            newPath.append(paths[column])
        return newPath

    def Refinement(self, paths):
        """ Make the paths more detailed """
        length = 0
        for path in paths:
            if len(path) > length:
                length = len(path)
        for path in paths:
            while len(path) < length:
                path.append(path[-1])
        total = 0.0
        num = 0
        for path in paths:
            for i in range(len(path) - 1):
                if path[i] != path[i + 1]:
                    total += math.sqrt(math.pow(path[i][0] - path[i + 1][0], 2) + math.pow(path[i][1] - path[i + 1][1], 2))
                    num += 1
        if num == 0:
            return paths
        pts = (int(total / num) + 1) // 4
        if pts == 0:
            return paths
        newPath = [list() for path in paths]
        for index, path in enumerate(paths):
            for i in range(len(path) - 1):
                newPath[index].append(path[i])
                stepX = (path[i + 1][0] - path[i][0]) / pts
                stepY = (path[i + 1][1] - path[i][1]) / pts
                for j in range(int(pts)):
                    newPath[index].append((newPath[index][-1][0] + stepX, newPath[index][-1][1] + stepY))
            newPath[index].append(path[-1])
        return newPath

    def GetRandomArrangement(self):
        # Random locations without colliding, only in simulation mode
        starts = list()
        for i in range(len(self.cars.keys())):
            inserted = False
            while not inserted:
                newX = self.bound.width * random() + self.bound.l
                newY = self.bound.height * random() + self.bound.u
                if self.NoCollision(starts, newX, newY):
                    starts.append((newX, newY, random() * math.pi))
                    inserted = True
                else:
                    pass
        for index, item in enumerate(self.cars.keys()):
            (self.cars[item].x, self.cars[item].y, self.cars[item].theta) = starts[index]

    def GetRandomGoals(self):
        # Random goals without colliding
        goals = list()
        for i in range(len(self.cars.keys())):
            inserted = False
            while not inserted:
                newX = self.bound.width * random() + self.bound.l
                newY = self.bound.height * random() + self.bound.u
                if self.NoCollision(goals, newX, newY):
                    goals.append((newX, newY))
                    inserted = True
                else:
                    pass
        return goals

    def NoCollision(self, l, x, y):
        # Check if collision occurs
        for obj in l:
            if utils.CheckCollosion(2 * utils.wheelBase, x, y, obj[0], obj[1]):
                return False
        return True
def _is_sim_mode() -> bool:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-s", dest="sim", action="store_true", default=False)
    args, _ = parser.parse_known_args()
    return bool(os.environ.get("sim", args.sim))

if __name__ == "__main__":
    if not _is_sim_mode():
        start_vision_pub_if_needed()
    else:
        print("[qt_gui] Simulation mode: skip starting vision_pub (camera).")

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
