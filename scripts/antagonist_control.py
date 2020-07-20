import numpy as np

class Setup:
    def __init__(self, x0, r, L, period):
        self.q1 = 0
        self.dq1 = 0
        self.q2 = 0
        self.dq2 = 0
        self.x = 0
        self.dx = 0

        self.x0 = x0
        self.r = r
        self.L = L
        self.period = period

    def update(self, in_buffer):
        self.dq1 = (in_buffer[3]/1E3 - self.q1)/self.period
        self.q1 = in_buffer[3]/1E3

        self.dq2 = (in_buffer[4]/1E3 - self.q2)/self.period
        self.q2 = in_buffer[4]/1E3

        self.dx = (in_buffer[2]/1E3 - self.x)/self.period
        self.qx = in_buffer[2]/1E3
    
    def lin_to_full_state(self,X):
        x = X[0,:]
        dx = X[1,:]
        #ddx = -a*omega*omega*np.sin(omega*t)

        x1 = x + self.x0
        dx1 = dx
        #ddx1 = ddx

        q1 = np.sqrt(2*self.L*x1 - x1*x1)/self.r
        dq1 = -(2*x1*dx1 - 2*self.L*dx1)/(2*self.r*np.sqrt(2*self.L*x1 - x1*x1))
        #ddq1 = -pow(2*x1*dx1 - 2*self.L*dx1,2)/(4*self.r*pow(2*self.L*x1 - x1*x1,3/2)) - (2*dx1*x1 + 2*x1*ddx1 - 2*self.L*ddx1)/(2*self.r*sqrt(2*self.L*x1 - x1*x1))
        
        x2 = -x + self.x0
        dx2 = -dx
        #ddx2 = -ddx

        q2 = np.sqrt(2*self.L*x2 - x2*x2)/self.r
        dq2 = -(2*x2*dx2 - 2*self.L*dx2)/(2*self.r*np.sqrt(2*self.L*x2 - x2*x2))
        #desired_state.ddpos_q2 = -pow(2*x2*dx2 - 2*self.L*dx2,2)/(4*self.r*pow(2*self.L*x2 - x2*x2,3/2)) - (2*dx2*x2 + 2*x2*ddx2 - 2*self.L*ddx2)/(2*self.r*sqrt(2*self.L*x2 - x2*x2));
        
        X_des = np.array([x,
                        dx,
                        q1,
                        dq1,
                        q2,
                        dq2])
        return X_des

def delta_x(setup,x,q1,q2):
    L = setup.L
    r = setup.r
    x0 = setup.x0
    x1_con = L - np.sqrt(L**2 - q1*q1*r**2)- x0
    x2_con = L - np.sqrt(L**2 - q2*q2*r**2)- x0
    delta_x = np.array([x - x1_con, -x2_con - x])
    return delta_x

def step_traj(setup,t):
    scale_ampl = 4
    scale_t = 1
    amplitude = 5

    x = (np.sign((scale_t*t)%(2*amplitude) - amplitude)*np.trunc((scale_t*t)%(2*amplitude)) - np.sign((scale_t*t)%(2*amplitude) - amplitude)*amplitude - amplitude/2)*scale_ampl

    dx = t*0
    X_lin = np.array([[x],
                     [dx]])
    X_des = setup.lin_to_full_state(X_lin)
    return X_des

def chirp_traj(setup,a0,a,w0,nu,t):

    omega = 2*np.pi*(w0 + nu*t)
    x0 = setup.x0

    r = setup.r
    L = setup.L

    x = a0+a*np.sin(omega*t)
    dx = a*(omega + 4*np.pi*nu*t)*np.cos(omega*t)

    X_lin = np.array([[x],
                     [dx]])

    X_des = setup.lin_to_full_state(X_lin)
    return X_des


def antagonist_control(setup,X_des):
    u_min = -5
    u_max = 5
    kp_lin = 0.0
    kd_lin = 0.0
    kp = 0.1
    kd = 0.01

    X = np.array([[setup.x],
                  [setup.dx],
                  [setup.q1],
                  [setup.dq1],
                  [setup.q2],
                  [setup.dq2]])

    E = X_des - X
    if kp_lin*E[0] + kd_lin*E[1]>0:
        K = np.array([[kp_lin, kd_lin, kp, kd, 0, 0],
                      [0, 0 ,0, 0, kp, kd]])
    else:
        K = np.array([[0, 0, kp, kd, 0, 0],
                      [-kp_lin, -kd_lin, 0, 0, kp, kd]])
    u = np.dot(K,E)
    u = np.clip(u,u_min,u_max)
    return u

