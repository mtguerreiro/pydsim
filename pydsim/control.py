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

        e = (ref - x[1]) / u

        u_pi = self.u_1 + self.kp * e + (self.dt * self.ki - self.kp) * self.e_1

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

        self.set_model(self.A, self.B, self.C, self.dt)


    def set_model(self, A, B, C, dt):
        self.Ad = np.eye(2) + self.dt * self.A
        self.Bd = self.dt * self.B


    def control(self, x, u, ref):

        x = x.reshape(-1, 1)
        
        x_u_0 = self.Ad @ x
        x_u_0_0 = self.Ad @ x_u_0
        x_u_0_1 = self.Ad @ x_u_0 + self.Bd * u
            
        x_u_1 = self.Ad @ x + self.Bd * u
        x_u_1_0 = self.Ad @ x_u_1
        x_u_1_1 = self.Ad @ x_u_1 + self.Bd * u

        #print(x_u_0)
        #print(x_u_0_0)
        #print(x_u_0_1)
        #print(x_u_1)
        #print(x_u_1_0)
        #print(x_u_1_1)

        J_u_0 = self.alpha * (ref - x_u_0[1, 0]) ** 2
        J_u_0_0 = J_u_0 + self.alpha * (ref - x_u_0_0[1, 0]) ** 2
        J_u_0_1 = J_u_0 + self.alpha * (ref - x_u_0_1[1, 0]) ** 2 + self.beta
        if J_u_0_0 < J_u_0_1:
            J_u_0 = J_u_0_0
        else:
            J_u_0 = J_u_0_1

        J_u_1 = self.alpha * (ref - x_u_1[1, 0]) ** 2 + self.beta
        J_u_1_0 = J_u_1 + self.alpha * (ref - x_u_1_0[1, 0]) ** 2
        J_u_1_1 = J_u_1 + self.alpha * (ref - x_u_1_1[1, 0]) ** 2 + self.beta 
        if J_u_1_0 < J_u_1_1:
            J_u_1 = J_u_1_0
        else:
            J_u_1 = J_u_1_1

        u_opt = 0
        if J_u_1 < J_u_0:
            u_opt = 1

        return u_opt
