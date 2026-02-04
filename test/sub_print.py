import zmq
#能持续打印：id x0 y0 ... x3 y3 就说明 ZMQ 通了。
ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:5556") #127.0.0.1：指代“本机”。
sub.setsockopt(zmq.SUBSCRIBE, b"")

while True:
    print(sub.recv_string())

