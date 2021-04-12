import math
import scipy
import numpy as np
import pyctl as ctl
import pydsim.utils as pydutils


class OL:

    def __init__(self):
        
        self.dc = None


    def _set_params(self, dc):
        
        self.dc = dc


    def set_initial_conditions(self, ini_conditions):
        
        self.dc = ini_conditions['dc']

    
    def meas(self, signals, i, j):

        sigs = None
        
        return sigs

    
    def control(self, sigs):

        return self.dc


class PI:

    def __init__(self):

        # Controller parameters
        self.dt = None

        # Gains
        self.kp = None
        self.ki = None

        # Controller states        
        self.e_1 = 0
        self.u_1 = 0


    def _set_params(self, kp, ki, dt):

        self.dt = dt

        self.kp = kp
        self.ki = ki

        self.a1 = 1
        self.b0 = 1 / 2 * (2 * self.kp + self.dt * self.ki)
        self.b1 = 1 / 2 * (self.dt * self.ki - 2 * self.kp)


    def set_initial_conditions(self, ini_conditions):
        
        self.u_1 = ini_conditions['u_1']
        self.e_1 = ini_conditions['e_1']


    def meas(self, signals, i, j):
        vc = signals._x[i, 1]
        ref = signals.v_ref[j]

        sigs = [vc, ref]

        return sigs


    def control(self, sigs):

        vc = sigs[0]
        ref = sigs[1]
        
        e = (ref - vc)
        
        u_pi = self.a1 * self.u_1 + self.b0 * e + self.b1 * self.e_1
        if u_pi > 1: u_pi = 1
        elif u_pi < 0: u_pi = 0
        
        self.e_1 = e
        self.u_1 = u_pi
        
        return u_pi


class PID:

    def __init__(self):

        # Controller parameters
        self.dt = None

        # Gains
        self.kp = None
        self.ki = None
        self.kd = None
        self.N = None

        # Controlle states
        self.e_1 = 0
        self.e_2 = 0
        self.u_1 = 0
        self.u_2 = 0
        

    def _set_params(self, kp, ki, kd, N, dt):

        self.dt = dt

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.N = N

        T = 2 / self.dt

        self.a0 = T**2 + T * N
        self.a1 = (-2 * T**2) / self.a0
        self.a2 = (T**2 - T * N) / self.a0
        
        self.b0 = (T**2 * (kp + N*kd) + T * (ki + N*kp) + N*ki) / self.a0
        self.b1 = (2 * (N + ki - T**2 * (kp + N*kd))) / self.a0
        self.b2 = (T**2 * (kp + N*kd) - T * (ki + N*kp) + N*ki) / self.a0
    

    def set_initial_conditions(self, u_1=0, u_2=0, e=0, e_1=0, e_2=0):
        
        self.u_1 = u_1
        self.u_2 = u_2
        self.e = e
        self.e_1 = e_1
        self.e_2 = e_2


    def meas(self, signals, i, j):

        vc = signals._x[i, 1]
        ref = signals.v_ref[j]

        sigs = [vc, ref]

        return sigs

        
    def control(self, sigs):

        vc = sigs[0]
        ref = sigs[1]
        
        e = (ref - vc)
        
        u_pid = -self.a1 * self.u_1 - self.a2 * self.u_2 + self.b0 * e + self.b1 * self.e_1 + self.b2 * self.e_2
        if u_pid > 1:
            u_pid = 1
        elif u_pid < 0:
            u_pid = 0

        self.e_2 = self.e_1
        self.e_1 = e
        self.u_2 = self.u_1
        self.u_1 = u_pid
        
        return u_pid

    
class MPC:

    def __init__(self, mpc_params):
        self.dt = mpc_params['dt']
        v_in = mpc_params['v_in']
        self.A = mpc_params['A']
        self.B = mpc_params['B'] * v_in
        self.C = mpc_params['C']

        self.alpha = mpc_params['alpha']
        self.beta = mpc_params['beta']

        try:
            n_ref = mpc_params['ref'].shape[0]
            ref = np.zeros(n_ref + mpc_params['n_step'])
            ref[:n_ref] = mpc_params['ref']
            ref[n_ref:] = mpc_params['ref'][-1]
            self.ref = ref
        except:
            self.ref = None
        
        #self.ref = mpc_params['ref']

        try:
            self.il_max = mpc_params['il_max']
        except:
            self.il_max = np.inf

        self.n_step = mpc_params['n_step']

        self.set_model(self.A, self.B, self.C, self.dt)

        self.__i = 0


    def set_model(self, A, B, C, dt):
        #self.Ad = np.eye(2) + dt * A
        #self.Bd = dt * B
        self.Ad, self.Bd, self.Cd, _, _ = scipy.signal.cont2discrete((A, B, C, 0), dt, method='bilinear')
    

    def pred_cost(self, x, u, ref):
        
        x_u_1 = self.Ad @ x + self.Bd * u
        if np.abs(x_u_1[0, 0]) >= self.il_max:
            j_u_1 = np.inf
        else:
            j_u_1 = self.alpha * (ref - x_u_1[1, 0]) ** 2# + self.beta * u ** 2

        return x_u_1, j_u_1
    
    
    def opt(self, x, u, ref, n_step):
        
        x_u_0, j_u_0 = self.pred_cost(x, 0, ref[0])
        if n_step != 1:
            u_0_opt, j_0_opt = self.opt(x_u_0, 1, ref[1:], n_step - 1)
            j_u_0 += j_0_opt

        x_u_1, j_u_1 = self.pred_cost(x, 1, ref[0])
        if n_step != 1:
            u_1_opt, j_1_opt = self.opt(x_u_1, 1, ref[1:], n_step - 1)
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

        if self.ref is None:
            vref = np.zeros(self.n_step)
            vref[:] = ref
        else:
            i_i = self.__i
            i_f = self.__i + self.n_step
            vref = self.ref[i_i:i_f]
            self.__i += 1

        #print(vref)
        u_opt, j_opt = self.opt(x, u, vref, self.n_step)
        
        return u_opt


class DMPC:

    def __init__(self):

        # Controller parameters
        self.dt = None
        self.v_in = None

        # Model - continuous and discrete
        self.Am = None
        self.Bm = None
        self.Cm = None

        self.Ad = None
        self.Bd = None
        self.Cd = None

        # MPC params and gains
        self.n_p = None
        self.n_c = None
        self.r_w = None

        self.K_x = None
        self.K_y = None
        self.M = None

        # Reference and index for changing reference
        self.ref = None
        self.__i = 0

        # Controller states
        self.u_1 = None
        self.x_1 = None


    def _set_params(self, A, B, C, dt, v_in, n_p, n_c, r_w, ref=None):
        
        self.v_in = v_in
        self.Am, self.Bm, self.Cm = A, B * v_in, C

        self.dt = dt
        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B * v_in, C, 0), dt, method='bilinear')
        self.Ad, self.Bd, self.Cd = Ad, Bd, Cd

        self.n_p, self.n_c, self.r_w = n_p, n_c, r_w
        
        # Sets MPC gains depending on ref
        dmpc_sys = ctl.mpc.System(Ad, Bd, Cd, n_p=n_p, n_c=n_c, r_w=r_w)
        if ref is None:
            self.ref = None
            self.__i = None
            Ky, Kmpc = dmpc_sys.opt_cl_gains()
            self.K_x = Kmpc[0, :-1]
            self.K_y = Kmpc[0, -1]
        else:
            n_ref = ref.shape[0]
            _ref = np.zeros(n_ref + n_p)
            _ref[:n_ref] = ref
            _ref[n_ref:] = ref[-1]
            self.ref = _ref
            self.__i = 0
            F, Phi = ctl.mpc.opt_matrices(dmpc_sys.A, dmpc_sys.B, dmpc_sys.C, n_p, n_c)
            R_bar = r_w * np.eye(n_c)
            M = np.linalg.inv(Phi.T @ Phi + R_bar) @ Phi.T
            self.M = M[0, :]
            Ky, Kmpc = dmpc_sys.opt_cl_gains()
            self.K_x = Kmpc[0, :-1]
            self.K_y = Kmpc[0, -1]

        self.x_1 = np.array([0.0, 0.0])
        self.u_1 = 0.0
        

    def meas(self, signals, i, j):

        x = signals._x[i]
        ref = signals.v_ref[j]

        sigs = [x, ref]

        return sigs

    
    def control(self, sigs):

        x = sigs[0]
        ref = sigs[1]

        if self.ref is None:
            dx = (x - self.x_1)
            du = -self.K_y * (x[1] - ref) + -self.K_x @ dx
            u_dmpc = du + self.u_1
            if u_dmpc > 1: u_dmpc = 1
            elif u_dmpc < 0: u_dmpc = 0
        else:
            i_i = self.__i
            i_f = self.__i + self.n_p
            Rs_bar = np.ones((self.n_p, 1))
            Rs_bar[:, 0] = self.ref[i_i:i_f]
            K_r = (self.M @ Rs_bar)
            dx = (x - self.x_1)
            du = K_r + -self.K_y * x[1] + -self.K_x @ dx
            u_dmpc = du[0] + self.u_1
            self.__i += 1

        self.x_1 = x
        self.u_1 = u_dmpc

        return u_dmpc


class SFB:
    
    def __init__(self):

        # Controller parameters
        self.dt = None
        self.v_in = None

        # Poles
        self.poles = None

        # Model and augmented model
        self.A = None
        self.B = None
        self.C = None

        self.Aa = None
        self.Ba = None

        # Gains
        self.Kx = None
        self.Ky = None

        # Controlle states
        self.e_1 = 0
        self.zeta_1 = 0

    def _set_params(self, A, B, C, poles, v_in, dt):

        self.poles = poles
        self.v_in = v_in
        self.dt = dt

        self.A = A
        self.B = B * v_in
        self.C = C

        # Augmented model
        Aa, Ba = self._aug_model(A, B * v_in, C)
        
        # Ackermann
        Kx = self._acker(Aa, Ba, poles)
        self.Kx = Kx[0, :-1]
        self.Kz = Kx[0, -1]


    def _aug_model(self, A, B, C):
        
        Aa = np.zeros((3,3))
        Ba = np.zeros((3,1))        

        Aa[:2, :2] = A
        Aa[2, :2] = C
        Ba[:2, 0] = B[:, 0]

        return Aa, Ba


    def _acker(self, Aa, Ba, p):

        c_eq = np.polymul(np.polymul([1, -p[0]], [1, -p[1]]).real, [1, -p[2]]).real

        Mc = np.zeros((3,3))
        Mc[:, 0] = Ba[:, 0]
        Mc[:, 1] = (Aa @ Ba)[:, 0]
        Mc[:, 2] = (Aa @ Aa @ Ba)[:, 0]

        Phi_d = c_eq[0] * Aa @ Aa @ Aa + c_eq[1] * Aa @ Aa + c_eq[2] * Aa + c_eq[3] * np.eye(3)

        Kx = np.array([[0, 0, 1]]) @ np.linalg.inv(Mc) @ Phi_d

        return Kx


    def meas(self, signals, i, j):
        x = signals._x[i]
        r = signals.v_ref[j]

        sigs = [x, r]
        
        return sigs
    

    def control(self, sigs):
        x = sigs[0]
        r = sigs[1]

        e = (r - x[1])
        zeta = self.zeta_1 + self.dt / 2 * (e + self.e_1)
        
        u_sfb = -self.Kx @ x + self.Kz * zeta

        self.zeta_1 = zeta
        self.e_1 = e
        
        return u_sfb

