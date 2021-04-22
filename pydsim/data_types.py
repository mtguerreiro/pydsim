import numpy as np
import scipy


class Signals:

    def __init__(self):

        self.x_ini = np.array([0.0, 0.0])
        self.t = None
        self.t_p = None
        self.x = None
        self._x = None
        self.v_in = None
        self.v_ref = None

        self.d = None
        self.pwm = None
        

    def _set_vectors(self, dt, t_pwm, t_sim):

        n = round(t_sim / dt)
        self.t = dt * np.arange(n)
        self.x = np.zeros((n, 2))
        self._x = np.zeros((n + 1, 2))
        self.pwm = np.zeros(n)
        self.d = np.zeros(n)

        n_cycles = round(t_sim / t_pwm)
        self.t_p = t_pwm * np.arange(n_cycles)
        self.v_in = np.zeros(n_cycles)
        self.v_ref = np.zeros(n_cycles)


class SimParams:
    
    def __init__(self, dt=None, t_sim=None, max_step=None):
        
        self.dt = dt
        self.t_sim = t_sim
        self.max_step = max_step

        
    def _set_dt(self, dt):
        self.dt = dt
        

    def _set_t_sim(self, t_sim):
        self.t_sim = t_sim
        

    def _set_max_step(self, max_step):
        self.max_step = max_step


class TwoPoleCircuit:
    
    def __init__(self, R, L, C, f_pwm):

        self.R = R
        self.L = L
        self.C = C
        
        if f_pwm is not None:
            self.f_pwm = f_pwm
            self.t_pwm = 1 / f_pwm
        else:
            self.f_pwm = None
            self.t_pwm = None


    def _get_params(self):

        return self.R, self.L, self.C, self.f_pwm
    

    def _set_f_pwm(self, f_pwm):
        
        self.f_pwm = f_pwm
        self.t_pwm = 1 / f_pwm


class SSModel:

    def __init__(self):

        self.A = None
        self.B = None
        self.C = None

        self.Ad = None
        self.Bd = None
        self.Cd = None

        self.dt = None
        

    def _set_model(self, A, B, C, dt=None):

        self.A, self.B, self.C = A, B, C

        if dt is not None:
            Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B, C, 0), dt, method='bilinear')
            self.Ad, self.Bd, self.Cd = Ad, Bd, Cd
            self.dt

    def _get_model(self):

        return (self.A, self.B, self.C)

    def _get_model_discrete(self):

        return (self.Ad, self.Bd, self.Cd, self.dt)

        
class BuckModel:
    
    def __init__(self):

        self._model = SSModel()

        self.A = None
        self.B = None
        self.C = None

        self.Ad = None
        self.Bd = None
        self.Cd = None

        self.dt = None
        

    def _set_model(self, R, L, C, dt):

        self.dt = dt

        # Continuous model
        A = np.array([[0,      -1/L],
                      [1/C,    -1/R/C]])
        
        B = np.array([[1/L],
                      [0]])
        
        C = np.array([0, 1])

        self.A, self.B, self.C = A, B, C

        self._model._set_model(A, B, C, dt)

        # Discrete Model
        Ad, Bd, Cd, _ = self._model._get_model_discrete()
        self.Ad, self.Bd, self.Cd = Ad, Bd, Cd
        
