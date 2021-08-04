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

        self.x_lp = np.array([0.0, 0.0])
        self.d_lp = 0
        

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
    
    def __init__(self, R, L, C, f_pwm, Rl=0., Rc=0., Rds=0.):

        self.R = R

        self.L = L
        self.Rl = Rl
        
        self.C = C
        self.Rc = Rc

        self.Rds = Rds
        
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

        self.x_lp = None
        self.u_lp = None
        

    def _set_model(self, A, B, C, dt=None, x_lp=None, u_lp=None):

        self.A, self.B, self.C = A, B, C
        self.dt = dt

        if dt is not None:
            Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B, C, 0), dt, method='zoh')
            self.Ad, self.Bd, self.Cd = Ad, Bd, Cd

        self.x_lp, self.u_lp = x_lp, u_lp
        

    def _get_model(self):

        return (self.A, self.B, self.C)
    

    def _get_model_discrete(self):

        return (self.Ad, self.Bd, self.Cd, self.dt)
    
        
class BuckModel:
    
    def __init__(self):
        
        self.A = None
        self.B = None
        self.C = None

        self.Ad = None
        self.Bd = None
        self.Cd = None

        self.dt = None
        

    def _set_model(self, R, L, C, dt=None, Rl=0, Rc=0, Rds=0):

        a11 = -(Rds + Rl) / L
        a12 = -1 / L

        a21 = (L - (Rds + Rl) * Rc * C) * R / ((R + Rc) * L * C)
        a22 = -(R * Rc * C + L) / ((R + Rc) * L * C)

        b11 = 1 / L
        b21 = R * Rc / ((R + Rc) * L)

        # Continuous model
        A = np.array([[a11, a12],
                      [a21, a22]])

        B = np.array([[b11],
                      [b21]])

        C = np.array([0, 1])

##        # Continuous model
##        A = np.array([[0,      -1/L],
##                      [1/C,    -1/R/C]])
##        
##        B = np.array([[1/L],
##                      [0]])
##        
##        C = np.array([0, 1])

        self.A, self.B, self.C = A, B, C

        # Discrete model
        if dt is not None:
            self.dt = dt
            Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B, C, 0), dt, method='zoh')
            self.Ad, self.Bd, self.Cd = Ad, Bd, Cd     


class BoostLinModel:
    
    def __init__(self):

        self.A = None
        self.B = None
        self.C = None

        self.Ad = None
        self.Bd = None
        self.Cd = None

        self.dt = None

        self.x_lp = None
        self.u_lp = None
        

    def _set_model(self, R, L, C, v_ref, v_in, dt=None):

        u_lp = 1 - v_in / v_ref
        il_lp = (v_ref / R) / (1 - u_lp)

        self.u_lp = u_lp
        self.x_lp = np.array([il_lp, v_ref])

        # Linearized continuos model
        Am = np.array([[0, -(1 - u_lp) / L], [(1 - u_lp) / C, -1 / R / C]])
        Bm = np.array([[v_in / (1 - u_lp) / L], [-v_in / ((1 - u_lp) * u_lp * R * C)]])
        Cm = np.array([0, 1])
        self.A, self.B, self.C = Am, Bm, Cm

        # Discrete Model
        if dt is not None:
            self.dt = dt
            Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((Am, Bm, Cm, 0), dt, method='bilinear')
            self.Ad, self.Bd, self.Cd = Ad, Bd, Cd
