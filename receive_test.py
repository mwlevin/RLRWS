

import datetime
import socket

HOST = ''
PORT = 10888
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind((HOST, PORT))
data = True

last_receive = None
number=1
current_time = datetime.datetime.now()
last_time = current_time

s.setblocking(False)


while True:
    current_time = datetime.datetime.now()
    if ((current_time-last_time).total_seconds())>=2:
        last_time = current_time
        newestData = None
        keepReceiving = True

        while keepReceiving:
            try:
                data, fromAddr = s.recvfrom(2048)
                if data:
                    newestData = data
            except socket.error as why:
                # keepReceiving = False
                break
                # if why.args[0] == "EWOULDBLOCK":
                #     keepReceiving = False
                # else:
                #     raise why
        # current_time = datetime.datetime.now()
        # if ((current_time-last_time).total_seconds())>=1/5:
        #     last_time = current_time
        if newestData:
            print('receive latest data', newestData.decode('utf-8'))




