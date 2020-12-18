import scipy
import numpy as np
import pydsim.utils as pydutils

class PI:

    def __init__(self, pi_params):

        self.kp = pi_params['kp']
        self.ki = pi_params['ki']
        self.dt = pi_params['dt']
        
        self.e_1 = 0
        self.u_1 = 0


    def set_params(self, pi_params):
        self.kp = pi_params['kp']
        self.ki = pi_params['ki']
        self.dt = pi_params['dt']


    def set_initial_conditions(self, ini_conditions):
        self.u_1 = ini_conditions['u_1']
        self.e_1 = ini_conditions['e_1']


    def control(self, x, u, ref):

        #dt = self.dt
        #kp = self.kp
        #ki = self.ki

        e = (ref - x[1]) / u

        u_pi = self.u_1 + self.kp * e + (self.dt * self.ki - self.kp) * self.e_1
        #u_pi = 1/2 * (2 * self.u_1 + (2 * kp + dt * ki) * e + (dt * ki - 2 * kp) * self.e_1)
        self.e_1 = e
        self.u_1 = u_pi
        
        return u_pi

class OL:

    def __init__(self, ol_params):
        self.dc = ol_params['dc']


    def set_params(self, ol_params):
        self.dc = ol_params['dc']


    def set_initial_conditions(self, ini_conditions):
        self.dc = ini_conditions['dc']


    def control(self, x, u, ref):

        return self.dc


class MPC:

    def __init__(self, mpc_params):
        self.A = mpc_params['A']
        self.B = mpc_params['B']
        self.C = mpc_params['C']
        self.dt = mpc_params['dt']

        self.alpha = mpc_params['alpha']
        self.beta = mpc_params['beta']

        self.n_step = mpc_params['n_step']

        self.set_model(self.A, self.B, self.C, self.dt)


    def set_model(self, A, B, C, dt):
        #self.Ad = np.eye(2) + dt * A
        #self.Bd = dt * B
        self.Ad, self.Bd, self.Cd, _, _ = scipy.signal.cont2discrete((A, B, C, 0), dt, method='bilinear')
    

    def pred_cost(self, x, u, ref):
        
        x_u_1 = self.Ad @ x + self.Bd * u
        j_u_1 = self.alpha * (ref - x_u_1[1, 0]) ** 2 + self.beta * u ** 2

        return x_u_1, j_u_1
    
    
    def opt(self, x, u, ref, n_step):

        x_u_0, j_u_0 = self.pred_cost(x, 0, ref)
        if n_step != 1:
            u_0_opt, j_0_opt = self.opt(x_u_0, u, ref, n_step - 1)
            j_u_0 += j_0_opt

        x_u_1, j_u_1 = self.pred_cost(x, u, ref)
        if n_step != 1:
            u_1_opt, j_1_opt = self.opt(x_u_1, u, ref, n_step - 1)
            j_u_1 += j_1_opt

        if j_u_0 < j_u_1:
            j_opt = j_u_0
            u_opt = 0
        else:
            j_opt = j_u_1
            u_opt = u
        
        return u_opt, j_opt


    def control(self, x, u, ref):

        x = x.reshape(-1, 1)

        u_opt, j_opt = self.opt(x, u, ref, self.n_step)

        return u_opt
