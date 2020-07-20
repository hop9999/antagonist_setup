import socket
import numpy as np
import os
import matplotlib.pyplot as plt
import math
from antagonist_control import *
from sklearn.metrics import mean_squared_error

def send_action(socket, addr, check, u1, u2):
    x = np.array([check,int(1000*u1),int(1000*u2)], dtype=np.int32)
    sock.sendto(x, addr)
    if(int_data[1] != 0):
        print(int_data[1])
        

if __name__ == "__main__":

    script_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    data_path = "C:/Users/admin/Documents/GitHub/antagonist_setup/data/"
    plots_path = os.path.join(script_dir, 'plots/')

    nu = 'sin'
    data_file_name = data_path + "exp_" + str(nu)

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
    sock.setblocking(0)
    sock.settimeout(0.05)
    #trajectory
    a0 = 0
    a = 20
    w0 = 0.
    nu = 0.01
    
    fs = 200
    T = 10
    N = int(T*fs)
    antagonist = Setup( 20, 0.71, 210, 5)
    data_array = np.zeros((N, 8))
    print(' *** DATA RECORDING *** ')
    for i in range(N):
        try:
            data, addr = sock.recvfrom(4*8)
            int_data = np.frombuffer(data, dtype=np.int32)
            antagonist.update(int_data)

            t = (int_data[0] - data_array[0, 0])/1E6
            #X_des = chirp_traj(antagonist,a0,a,w0,nu,t)
            X_des = step_traj(antagonist,t)
            #print(X_des)
            '''X_des = np.array([[0],
                    [0],
                    [0],
                    [0],
                    [0],
                    [0]])'''
            u = antagonist_control(antagonist,X_des)
            #print(u)
            u_preload = 0.
            send_action(sock, addr, int_data[0], u[0]+u_preload, u[1]+u_preload)
            
            data_array[i, 0:8] = int_data
        except socket.timeout:
            print("except")
        

    print('\n\n *** EXPERIMENT IS OVER ***')


    time = (data_array[:, 0] - data_array[0, 0])/1E6
    check = data_array[:, 1]/1E3
    x = data_array[:, 2]/1E3
    q1 = data_array[:, 3]/1E3
    q2 = data_array[:, 4]/1E3
    u1 = data_array[:, 5]/1E3
    u2 = data_array[:, 6]/1E3
    force = data_array[:, 7]/1E3

    np.savetxt(data_file_name+'.csv', np.transpose(data_array), delimiter=',')  # X is an array

    print(' Data saved to:\n' + data_file_name+'.csv\n\n')

    traj = step_traj(antagonist,time)
    print(traj)
    x_des = traj[0,:][0]
    q1_des = traj[2,:][0]
    q2_des = traj[4,:][0]

    print(mean_squared_error(x, x_des, squared=False))

    fig, axs = plt.subplots(3)
    axs[0].plot(time[1:],x[1:],label='x')
    axs[0].plot(time[1:],x_des[1:],label='x_des')
    axs[0].set_ylabel('mm')
    axs[0].legend(loc='lower right')
    axs[1].plot(time[1:],q1[1:],label='q1')  
    axs[1].plot(time[1:],q2[1:],label='q2')   
    axs[1].plot(time[1:],q1_des[1:],label='q1_des')  
    axs[1].plot(time[1:],q2_des[1:],label='q2_des')   
    axs[1].set_ylabel('mm')
    axs[1].set_xlabel('time, s')
    axs[1].legend(loc='lower right')  
    #axs[2].plot(time[1:],x[1:],label='x')
    axs[2].plot(time[1:],u1[1:],label='x1_con')  
    axs[2].plot(time[1:],u2[1:],label='x2_con')   
    axs[2].set_ylabel('mm')
    axs[2].set_xlabel('time, s')
    axs[2].legend(loc='lower right')

    #  Определяем внешний вид линий основной сетки:
    axs[0].grid(which='major',
            color = 'k', 
            linewidth = 1)
    axs[1].grid(which='major',
            color = 'k', 
            linewidth = 1)
    axs[2].grid(which='major',
            color = 'k', 
            linewidth = 1)
    plt.show()




