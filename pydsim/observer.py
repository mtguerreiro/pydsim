import scipy
import numpy as np
import pyctl as ctl
import pyoctl.opt as octl
import pydsim.utils as pydutils


class Luenberger:

    def __init__(self):

        # Observer parameters
        self.dt = None
        self.poles = None
        
        # Plant and observer models
        self.A = None
        self.B = None
        self.C = None

        self.Ao = None
        self.Bo = None
        self.Co = None

        self.Aod = None
        self.Bod = None
        self.Cod = None

        # Observer gains
        self.Ko = None

        # Observer states
        self.x_bar_k = np.zeros(2, dtype=np.float)
        self.x_bar_k_1 = 0
        self.x_obs = []
        

    def _set_params(self, A, B, C, poles, dt):

        self.poles = poles
        self.dt = dt
        
        self.A, self.B, self.C = A, B, C
        n = A.shape[0]

        C = np.array([C])

        # Observer gains
        sp = scipy.signal.place_poles(A.T, C.T, poles)
        Ko = sp.gain_matrix.T
        self.Ko = Ko

        # Continuous-time observer system. We consider two inputs for the
        # observer system: the control signal applied to the plant and the
        # output of the plant
        Ao = A - Ko @ C

        Bo = np.zeros((n, 2))
        Bo[:, 0] = B[:, 0]
        Bo[:, 1] = Ko[:, 0]

        Co = C

        self.Ao, self.Bo, self.Co = Ao, Bo, Co

        # Discrete system        
        Aod, Bod, Cod, _, _ = scipy.signal.cont2discrete((Ao, Bo, Co, 0), dt, method='bilinear')
        self.Aod, self.Bod, self.Cod = Aod, Bod, Cod


    def _save_state(self, x):
        
        self.x_obs.append(self.x_bar_k)


    def get_states(self):
        
        x_obs = np.array(self.x_obs)
        n = (x_obs.shape[0], x_obs.shape[1])
        
        return x_obs.reshape(n)


    def get_current_state(self):
        
        return self.x_obs[-1]

    
    def estimate(self, y, u):
        
        uo = np.array([u, y])        
        self.x_bar_k_1 = self.Aod @ self.x_bar_k + self.Bod @ uo
        
        self._save_state(self.x_bar_k)

        self.x_bar_k = self.x_bar_k_1
        
        return self.x_bar_k_1
        
