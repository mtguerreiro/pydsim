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


    def control(self, e):

        u = self.u_1 + self.kp * e + (self.dt * self.ki - self.kp) * self.e_1

        self.e_1 = e
        self.u_1 = u
        
        return u

class OL:

    def __init__(self, ol_params):
        self.dc = ol_params['dc']


    def set_params(self, ol_params):
        self.dc = ol_params['dc']


    def set_initial_conditions(self, ini_conditions):
        self.dc = ini_conditions['dc']


    def control(self, e):

        return self.dc
