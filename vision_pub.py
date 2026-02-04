# Responsible for opening the camera, capturing frames, and sending
# ArUco marker id plus four corner coordinates via ZMQ broadcast.
# Send format: id x0 y0 x1 y1 x2 y2 x3 y3
# Capture rate is adjustable; default is 30 FPS (30 frames/sec).
# The send frequency roughly matches the capture rate: send one message
# per detected marker per frame. Example:
# If there are 3 markers in the view and 30 FPS, that's ~90 messages/sec.

import time
import zmq# used to send data over the network
import cv2
import os

# =====================
# ZMQ PUB config
# =====================
PUB_IP = "0.0.0.0" # listen on all local interfaces so LAN machines can connect
PUB_PORT = 5556  # align with utils.zmqPublisherPort; like a room number
# 1280*720
# =====================
# Camera config
# =====================
CAM_INDEX = 0  # default first camera
FPS_LIMIT = 30

def main():
    # ZMQ PUB
    ctx = zmq.Context.instance() # create a ZMQ context
    pub = ctx.socket(zmq.PUB) # declare a PUB (publisher) socket
    pub.bind(f"tcp://{PUB_IP}:{PUB_PORT}") # bind address and port to start broadcasting
    print(f"[PUB] bound on tcp://{PUB_IP}:{PUB_PORT}")

    # Camera
    cap = cv2.VideoCapture(CAM_INDEX) # open the camera
    if not cap.isOpened():
        raise RuntimeError("Camera open failed.")
    # # force set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, FPS_LIMIT)
    # print actual camera resolution
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("First frame read failed.")
    h, w = frame.shape[:2]
    print(f"Camera confirmed: {w} x {h}", flush=True)

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)

    last = 0.0
    while True:
        ret, frame = cap.read() # grab a frame from the camera
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale; detection is faster
        corners_list, ids, _ = detector.detectMarkers(gray) # detect markers

        if ids is not None:
            # draw all detected ArUco markers at once
            cv2.aruco.drawDetectedMarkers(frame, corners_list, ids)

            # then send each detected marker via ZMQ
            for corners, tag_id in zip(corners_list, ids.flatten()):
                c = corners[0]
                msg = f"{int(tag_id)} {c[0][0]} {c[0][1]} {c[1][0]} {c[1][1]} {c[2][0]} {c[2][1]} {c[3][0]} {c[3][1]}"
                pub.send_string(msg)


        cv2.imshow("vision_pub", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        # simple rate limiting: if running too fast, sleep to respect FPS limit
        now = time.time()
        dt = now - last
        if dt < 1.0 / FPS_LIMIT:
            time.sleep(max(0.0, 1.0 / FPS_LIMIT - dt))
        last = time.time()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
