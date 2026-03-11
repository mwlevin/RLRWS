

import datetime
import socket
import pickle as pkl
from cav_data import *

HOST = ''
PORT = 10888
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind((HOST, PORT))
data = True

last_receive = None
number=1
current_time = datetime.datetime.now()
last_time = current_time
while True:
    data, fromAddr = s.recvfrom(2048)
    data_org = pkl.loads(data)
    print('spd',data_org.spd)
    
    # newestData = None
    # keepReceiving = True

    
    
    
    # while keepReceiving:
    #     current_time = datetime.datetime.now()
        
    #     if ((current_time-last_time).total_seconds())>=1/2:
    #         last_time = current_time
    #         try:
    #             data, fromAddr = s.recvfrom(2048)
    #             if data:
    #                 newestData = data
    #         except socket.error as why:
    #             if why.args[0] == "EWOULDBLOCK":
    #                 keepReceiving = False
    #             else:
    #                 raise why
    #     if (newestData):
    #         print('receive latest data', data.decode('utf-8'))
    # number+=1
    # s.sendto(newestData,fromAddr)
    
    # data,address = s.recvfrom(2048)
    # current_time = datetime.datetime.now()
    # if data == b'bye':
    #     break
    # if last_receive is not None:
    #     print('Received String:',data.decode('utf-8'))
    #     print('time difference', (current_time - last_receive).total_seconds())
    #     last_receive = current_time
    # else:
    #     last_receive = current_time
    # s.sendto(data,address)
s.close()