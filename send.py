import socket
import datetime
from cav_data import *
import pickle as pkl

start_time = datetime.datetime.now()
HOST = 'localhost'
PORT = 10888
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data = '你好！'

frequency = 10

previous_ping_time = start_time
number = 1
while True:
    data = "你好"+str(number)
    current_time = datetime.datetime.now()
    if ((current_time-previous_ping_time).total_seconds())>=1/frequency:
        cav_test_data = CavData(10,10,10)
        cav_test_pkl = pkl.dumps(cav_test_data)
        s.sendto(cav_test_pkl,(HOST,PORT))
        # s.sendto(data.encode('utf-8'),(HOST,PORT))
        # print('send data: ', data)
        if data == 'bye':
            break
        previous_ping_time = current_time
        number+=1
    # data, addr = s.recvfrom(1024)
    # print('Recv from server:\n',data.decode('utf-8'))
    # data = input('Please input a info:\n')

s.close()