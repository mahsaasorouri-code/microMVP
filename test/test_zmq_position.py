import time
import positionZMQSub

print("Starting ZMQ subscriber...")
positionZMQSub._initialize_zmq()

while True:
    data = positionZMQSub._get_all_car_position_data()
    if data:
        print("Current carPosiDict:")
        for k, v in data.items():
            print(f"  ID {k}: {v}")
        print("-" * 30)
    else:
        print("No data yet...")
    time.sleep(1)
