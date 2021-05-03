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
        self.x_bar_k_1 = np.zeros(2, dtype=np.float)
        self.x_obs = [np.zeros(2, dtype=np.float)]
        

    def _set_params(self, params):

        A, B, C = params['A'], params['B'], params['C']
        poles, dt = params['poles'], params['dt']
        
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
        
        self.x_obs.append(x)


    def get_states(self):
        
        x_obs = np.array(self.x_obs[:-1])
        n = (x_obs.shape[0], x_obs.shape[1])
        
        return x_obs.reshape(n)


    def get_current_state(self, y):

        return self.x_bar_k_1

    
    def estimate(self, y, u):
        
        uo = np.array([u, y])        
        self.x_bar_k_1 = self.Aod @ self.x_bar_k + self.Bod @ uo
        
        self.x_bar_k = self.x_bar_k_1

        self._save_state(self.x_bar_k)

        return self.x_bar_k_1


class LuenbergerC:

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
        self.x_tilde_k = np.zeros(2, dtype=np.float)
        self.x_obs = []
        

    def _set_params(self, params):

        A, B, C = params['A'], params['B'], params['C']
        poles, dt = params['poles'], params['dt']
        
        self.poles = poles
        self.dt = dt
        
        self.A, self.B, self.C = A, B, C
        n = A.shape[0]

        C = np.array([C])

        # Observer gains
        sp = scipy.signal.place_poles(A.T, (C @ A).T, poles)
        Ko = sp.gain_matrix.T
        self.Ko = Ko

        # Describe here current state observer
        Ao = A - Ko @ C @ A

        Bo = B[:, 0]
        #Bo = np.zeros((n, 2))
        #Bo[:, 0] = B[:, 0]
        #Bo[:, 1] = (A @ Ko)[:, 0]

        Co = C

        self.Ao, self.Bo, self.Co = Ao, Bo, Co

        # Discrete system        
        Aod, Bod, Cod, _, _ = scipy.signal.cont2discrete((Ao, Bo, Co, 0), dt, method='bilinear')
        self.Aod, self.Bod, self.Cod = Aod, Bod, Cod


    def _save_state(self, x):
        
        self.x_obs.append(x)


    def get_states(self):
        
        x_obs = np.array(self.x_obs)
        n = (x_obs.shape[0], x_obs.shape[1])
        
        return x_obs.reshape(n)


    def get_current_state(self, y):

        self.x_tilde_k = self.x_bar_k + self.Ko @ (y - self.Cod @ self.x_bar_k)

        self._save_state(self.x_tilde_k)
        
        return self.x_tilde_k

    
    def estimate(self, y, u):

        self.x_bar_k_1 = self.Aod @ self.x_tilde_k + self.Bod * u     
        
        self.x_bar_k = self.x_bar_k_1
        
        return self.x_bar_k_1

    
class DisturbanceObs:
    
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
        self.x_bar_k = np.zeros(3, dtype=np.float)
        self.x_bar_k_1 = 0
        self.x_obs = [np.zeros(3, dtype=np.float)]


    def _set_params(self, params):

        A, B, C = params['A'], params['B'], params['C']
        poles, dt = params['poles'], params['dt']
        
        self.poles = poles
        self.dt = dt
        
        self.A, self.B, self.C = A, B, C
        n = A.shape[0]

        # Gains for disturbance observer      
        Aao = np.zeros((3, 3))
        Aao[:2, :2] = A
        Aao[:2, 2] = B[:, 0]
        Aao[-1, -1] = 1

        Cao = np.zeros(3)
        Cao[:2] = C
        Cao = np.array([Cao])

        sp = scipy.signal.place_poles(Aao.T, Cao.T, poles)
        Ko = sp.gain_matrix.T
        self.Ko = Ko

        # Now builds continuous-time observer system with observer gains
        Ao = Aao - Ko @ Cao
        #Ao = Aao - Ko * Cao
        
        Bo = np.zeros((3,2))
        Bo[:2, 0] = B[:, 0]
        Bo[0:, 1] = Ko[:, 0]

        Co = np.zeros((1, 3))
        Co[0, :2] = C

        self.Ao, self.Bo, self.Co = Ao, Bo, Co

        # Discrete-time observer
        Aod, Bod, Cod, _, _ = scipy.signal.cont2discrete((Ao, Bo, Co, 0), dt, method='bilinear')
        self.Aod, self.Bod, self.Cod = Aod, Bod, Cod    


    def _save_state(self, x):
        
        self.x_obs.append(x)


    def get_states(self):
        
        x_obs = np.array(self.x_obs[:-1])
        n = (x_obs.shape[0], x_obs.shape[1])
        
        return x_obs.reshape(n)


    def get_current_state(self, y):
        
        return self.x_bar_k[:-1]
    

    def estimate(self, y, u):
        
        uo = np.array([u, y])
        self.x_bar_k_1 = self.Aod @ self.x_bar_k + self.Bod @ uo
        
        self.x_bar_k = self.x_bar_k_1

        self._save_state(self.x_bar_k)

        return self.x_bar_k_1[:-1]
  

class ESO:

    def __init__(self):

        # Observer parameters
        self.dt = None
        self.b1 = None
        self.b2 = None
        self.b3 = None
        
        # Observer model
        self.Ao = None
        self.Bo = None

        self.Aod = None
        self.Bod = None
        
        # Observer states
        self.x_tilde_k = np.zeros(3, dtype=np.float)
        self.x_tilde_k_1 = np.zeros(3, dtype=np.float)
        self.x_obs = [np.zeros(3, dtype=np.float)]

        
    def _set_params(self, params):

        b1, b2, b3 = params['b1'], params['b2'], params['b3']
        dt = params['dt']

        self.dt = dt
        self.b1, self.b2, self.b3 = b1, b2, b3
        
        # Observer model
        Ao = np.array([[-b1, 1, 0],
                       [-b2, 0, 1],
                       [-b3, 0, 0]])

        Bo = np.array([b1, b2, b3])

        self.Ao, self.Bo = Ao, Bo

        # Discrete-time observer
        Aod, Bod, Cod, _, _ = scipy.signal.cont2discrete((Ao, Bo, np.zeros(3), 0), dt, method='bilinear')
        self.Aod, self.Bod = Aod, Bod

    
    def _save_state(self, x):
        
        self.x_obs.append(x)


    def get_states(self):
        
        x_obs = np.array(self.x_obs[:-1])
        n = (x_obs.shape[0], x_obs.shape[1])
        
        return x_obs.reshape(n)


    def get_current_state(self, y):
        
        return self.x_tilde_k[:-1]
    

    def estimate(self, y, u):
        
        self.x_tilde_k_1 = self.Aod @ self.x_tilde_k + self.Bod * y
        
        self.x_tilde_k = self.x_tilde_k_1

        self._save_state(self.x_tilde_k)

        return self.x_tilde_k_1[:-1]
