import sys
import zmq
import time
import threading
from threading import Thread
import copy

import utils

# Dictionary
carPosiDict = dict() # create a dictionary to store: {carID: position_string}
# The carPosiDict maps `car id` to the four-corner coordinate string.

# Create a threading lock for safe access to dictionary
lock = threading.Lock()
import traceback
print("initialize_zmq called\n", "".join(traceback.format_stack(limit=6)))
# Continuously pull ZMQ data in a background thread and update carPosiDict
def _pull_zmq_data():
    #Connect to zmq publisher 
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket # subscribe to all messages

    print(f"Collecting update from server: tcp://{utils.zmqPublisherIP}:{utils.zmqPublisherPort}")
    socket.connect ("tcp://%s:%s" %(utils.zmqPublisherIP,
        utils.zmqPublisherPort))
    print ("Connected...")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    # Continuous update of car position, if available
    while True:     # receive raw string, e.g. "1 100 200..."
        msg = socket.recv_string()          # receive as str
        carID_str, rest = msg.split(" ", 1) # split into "id" and the rest coordinate string
        with lock:
            carPosiDict[int(carID_str)] = rest


# function to sense the tag of all cars at startup
# This function usually runs once at program start to determine how many cars
# are present on the field.
# When executed it:
# 1) updates the global list `utils.carInfo` with any discovered car IDs.
# 2) completes an initialization sweep so the program knows which tags exist.
def _pull_zmq_data_once():
    #Connect to zmq publisher 
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket

    print(f"Collecting update from server: tcp://{utils.zmqPublisherIP}:{utils.zmqPublisherPort}")
    socket.connect ("tcp://%s:%s" %(utils.zmqPublisherIP,
        utils.zmqPublisherPort))
    print ("Connected...")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    # Continuous update of car position, if available
    flag = True
    count = 0
    while flag:
        msg = socket.recv_string()              # receive the whole line as a string
        carID_str, rest = msg.split(" ", 1)     # split once: id first, then coordinate string
        carIDI = int(carID_str)

        with lock:
            if (carIDI, carIDI) not in utils.carInfo:
                utils.carInfo.append((carIDI, carIDI))
            else:
                count += 1

        if count == 10:
            flag = False

    # print utils.carInfo
    socket.close()
        
# Get a copy of all car position data
def _get_all_car_position_data():
    with lock:
        tempData = copy.deepcopy(carPosiDict)
    return tempData
        
# Get the position data for a specified car
def _get_car_position_data(carID):
    tempData = ""
    with lock:
        if carPosiDict.has_key(carID):
            tempData = carPosiDict[carID]
    return tempData

_t = None
_zmq_started = False

def _initialize_zmq():
    global _t, _zmq_started
    if _zmq_started:
        return  
    _zmq_started = True

    _t = Thread(target=_pull_zmq_data, daemon=True)
    _t.start()
    time.sleep(0.3)


def _stop_zmq():
    return
