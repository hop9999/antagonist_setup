import socket
import numpy as np
#from matplotlib import pyplot as plt  # Import plotting routines
import os  # Directory management and
import matplotlib.pyplot as plt
import math

class Setup:
    def __init__(self, period):
        self.q1 = 0
        self.dq1 = 0
        self.q2 = 0
        self.dq2 = 0
        self.x = 0
        self.dx = 0
        self.period = period

    def update(self, in_buffer):
        self.dq1 = (int_data[3]/1E3 - self.q1)/self.period
        self.q1 = int_data[3]/1E3

        self.dq2 = (int_data[4]/1E3 - self.q2)/self.period
        self.q2 = int_data[4]/1E3

        self.dx = (int_data[2]/1E3 - self.x)/self.period
        self.qx = int_data[2]/1E3

def pd_control(q,dq):
    q_des = 160
    kp = 0.1
    kd = 0.01
    u = kp*(q_des - q) + kd*(0 - dq)
    return u

def antagonist_control(setup):
    q_des = 100
    dq_des = 0
    kp = 0.1
    kd = 0.01
    K = np.array([[kp, kd, 0, 0],
                  [0, 0, kp, kd]])
    E = np.array([[q_des - setup.q1],
                  [dq_des - setup.dq1],
                  [q_des - setup.q2],
                  [dq_des - setup.dq2]])
    u = np.dot(K,E)
    return u

def send_action(socket, addr, check, u1, u2):
    x = np.array([check,int(1000*u1),int(1000*u2)], dtype=np.int32)
    sock.sendto(x, addr)
    if(int_data[1] != 0):
        print(int_data[1])
        

if __name__ == "__main__":

    SEND_IP = "192.168.3.3"
    UDP_IP = "192.168.3.2"
    UDP_PORT = 8151

    sock = socket.socket(socket.AF_INET,  # Internet
                        socket.SOCK_DGRAM)  # UDP

    print('\n *** CONNECTION *** \n'+
        ' Target IP: ' +UDP_IP + '\n'
        ' PORT: ' + str(UDP_PORT))

    sock.bind((UDP_IP, UDP_PORT))
    print(' Connected successfully \n')

    fs = 500
    T = 5
    N = int(T*fs)
    antagonist = Setup(1./fs)
    data_array = np.zeros((N, 11))

    print(' *** DATA RECORDING *** ')
    for i in range(N):
        
        #print(" Progress: {}%".format(int(100 * (i+1) / N)), end="\r", flush=True)
        data, addr = sock.recvfrom(4*8)  # buffer size is 1024 bytes
        int_data = np.frombuffer(data, dtype=np.int32)
        antagonist.update(int_data)
        u = antagonist_control(antagonist)
        send_action(sock, addr, int_data[0], u[0], u[1])
        
        
        data_array[i, 0:8] = int_data
        data_array[i, 8] = antagonist.dx
        data_array[i, 9] = antagonist.dq1
        data_array[i, 10] = antagonist.dq2

    print('\n\n *** EXPERIMENT IS OVER ***')


    time = (data_array[:, 0] - data_array[0, 0])/1E6
    check = data_array[:, 1]/1E3
    lin_pos = data_array[:, 2]/1E3
    q1_pos = data_array[:, 3]/1E3
    q2_pos = data_array[:, 4]/1E3
    q1_cur = data_array[:, 5]/1E3
    q2_cur = data_array[:, 6]/1E3
    force = data_array[:, 7]/1E3
    dx = data_array[:, 8]/1E3
    dq1 = data_array[:, 9]/1E3
    dq2 = data_array[:, 10]/1E3

    plt.plot(time[1:],lin_pos[1:],label='x')
    plt.plot(time[1:],dq1[1:],label='dq1')
    plt.plot(time[1:],q1_pos[1:],label='q1')    
    plt.plot(time[1:],dq2[1:],label='dq2')
    plt.plot(time[1:],q2_pos[1:],label='q2')

    plt.legend(loc='lower right')
    plt.grid()
    plt.show()

