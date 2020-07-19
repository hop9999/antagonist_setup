import numpy as np
import matplotlib.pyplot as plt
from antagonist_control import *

data_array = np.genfromtxt('/home/oleg/antagonist_control/data/chirp5.csv', delimiter=',')

time = (data_array[0, :] - data_array[0, 0])/1E6

x = data_array[2, :]/1E3
q1 = data_array[3, :]/1E3
q2 = data_array[4, :]/1E3
u1 = data_array[5, :]/1E3
u2 = data_array[6, :]/1E3

antagonist = Setup( 25, 0.71, 230, 5)

a0 = 0
a = 20
w0 = 0.0
nu = 0.05

traj = chirp_traj(antagonist,a0,a,w0,nu,time)
x_des = traj[0,:][0]
q1_des = traj[2,:][0]
q2_des = traj[4,:][0]
print(x_des)

deltax = delta_x(antagonist,x,q1,q2)
delta_x1 = deltax[0,:]
delta_x2 = deltax[1,:]

fig, axs = plt.subplots(3)
axs[0].plot(time[1:],x[1:],label='x')
axs[0].plot(time[1:],x_des[1:],'--', label='x_des')
axs[0].set_ylabel('mm')
axs[0].legend(loc='lower right')
axs[1].plot(time[1:],q1[1:],label='q1')  
axs[1].plot(time[1:],q2[1:],label='q2')   
axs[1].plot(time[1:],q1_des[1:],'--',label='q1_des')  
axs[1].plot(time[1:],q2_des[1:],'--',label='q2_des')   
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

