import numpy as np
import pydsim.utils as pydutils

class PI:

    def __init__(self, kp=None, ki=None, dt=None):

        self.kp = kp
        self.ki = ki
        self.dt = dt
        
        self.e_1 = 0
        self.u_1 = 0


    def set_params(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt


    def set_initial_conditions(self, u_1, e_1):
        self.u_1 = u_1
        self.e_1 = e_1


    def control(self, e):

        u = self.u_1 + self.kp * e + (self.dt * self.ki - self.kp) * self.e_1

        self.e_1 = e
        self.u_1 = u
        
        return u
