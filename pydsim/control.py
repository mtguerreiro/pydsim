import math
import scipy
import numpy as np
import pyctl as ctl
import pydsim.utils as pydutils

class PI:

    def __init__(self, pi_params):

        #self.kp = pi_params['kp']
        #self.ki = pi_params['ki']
        #self.dt = pi_params['dt']

        self.set_params(pi_params)
        
        self.e_1 = 0
        self.u_1 = 0


    def set_params(self, pi_params):
        self.kp = pi_params['kp']
        self.ki = pi_params['ki']
        self.dt = pi_params['dt']

        self.a1 = 1
        self.b0 = 1 / 2 * (2 * self.kp + self.dt * self.ki)
        self.b1 = 1 / 2 * (self.dt * self.ki - 2 * self.kp)
        #self.a1 = 1
        #self.b0 = 1 / 2 * (2 * self.kp + self.dt * self.ki)
        #self.b1 = 1 / 2 * (self.dt * self.ki)
        
        #self.a1 = 1
        #self.b0 = self.kp
        #self.b1 = (self.dt * self.ki - self.kp)
        

    def set_initial_conditions(self, ini_conditions):
        self.u_1 = ini_conditions['u_1']
        self.e_1 = ini_conditions['e_1']


    def control(self, x, u, ref):
        
        e = (ref - x[1]) / u
        
        u_pi = self.a1 * self.u_1 + self.b0 * e + self.b1 * self.e_1
        self.e_1 = e
        self.u_1 = u_pi
        
        return u_pi


class PID:

    def __init__(self, pid_params):

        #self.kp = pi_params['kp']
        #self.ki = pi_params['ki']
        #self.dt = pi_params['dt']

        self.set_params(pid_params)

        self.e_1 = 0
        self.e_2 = 0
        self.u_1 = 0
        self.u_2 = 0
        

    def set_params(self, pid_params):
        self.kp = pid_params['kp']
        self.ki = pid_params['ki']
        self.kd = pid_params['kd']
        self.N = pid_params['N']
        self.dt = pid_params['dt']
        self.sat = pid_params['sat']

        kp = self.kp
        ki = self.ki
        kd = self.kd
        N = self.N
        T = 2 / self.dt

        self.a0 = T**2 + T * N
        self.a1 = -(-2 * T**2) / self.a0
        self.a2 = -(T**2 - T * N) / self.a0
        
        self.b0 = (T**2 * (kp + N*kd) + T * (ki + N*kp) + N*ki) / self.a0
        self.b1 = (2 * (N + ki - T**2 * (kp + N*kd))) / self.a0
        self.b2 = (T**2 * (kp + N*kd) - T * (ki + N*kp) + N*ki) / self.a0
        
        #a = self.kp + self.N * self.kd
        #b = self.ki + self.N * self.kp
        #c = self.N * self.ki

        #self.a1 = -(self.dt * self.N - 2)
        #self.a2 = -(1 - self.dt * self.N)
        #self.b0 = a
        #self.b1 = self.dt * b - 2 * a
        #self.b2 = a**2 + self.dt**2 * c - self.dt * b


    def set_initial_conditions(self, u_1=0, u_2=0, e=0, e_1=0, e_2=0):
        self.u_1 = u_1
        self.u_2 = u_2
        self.e = e
        self.e_1 = e_1
        self.e_2 = e_2

    
    def control(self, x, u, ref):
        
        e = (ref - x[1]) / u
        
        u_pid = self.a1 * self.u_1 + self.a2 * self.u_2 + self.b0 * e + self.b1 * self.e_1 + self.b2 * self.e_2

        if self.sat is True:
            if u_pid > 1:
                u_pid = 1
            elif u_pid < 0:
                u_pid = 0

        self.e_2 = self.e_1
        self.e_1 = e
        self.u_2 = self.u_1
        self.u_1 = u_pid

        #print(u_pid, self.u_1, self.u_2, e, self.e_1, self.e_2)
        
        return u_pid

        
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
            u_0_opt, j_0_opt = self.opt(x_u_0, u, ref[1:], n_step - 1)
            j_u_0 += j_0_opt

        x_u_1, j_u_1 = self.pred_cost(x, u, ref[0])
        if n_step != 1:
            u_1_opt, j_1_opt = self.opt(x_u_1, u, ref[1:], n_step - 1)
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
        
        return u_opt / u


class DMPC:

    def __init__(self, dmpc_params):
        dt = dmpc_params['dt']
        v_in = dmpc_params['v_in']
        Am = dmpc_params['A']
        Bm = dmpc_params['B'] * v_in
        Cm = dmpc_params['C']
        self.Am = Am; self.Bm = Bm; self.Cm = Cm; self.dt = dt; self.v_in = v_in

        n_p = dmpc_params['n_p']
        n_c = dmpc_params['n_c']
        r_w = dmpc_params['r_w']
        self.n_p = n_p; self.n_c = n_c; self.r_w = r_w

        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((Am, Bm, Cm, 0), dt, method='bilinear')
        self.Ad = Ad; self.Bd = Bd; self.Cd = Cd

        self.__i = 0

        try:
            n_ref = dmpc_params['ref'].shape[0]
            ref = np.zeros(n_ref + n_p)
            ref[:n_ref] = dmpc_params['ref']
            ref[n_ref:] = dmpc_params['ref'][-1]           
            self.ref = ref
        except:
            ref = None
            self.ref = ref

        if ref is None:
            self.dmpc_sys = ctl.mpc.System(Ad, Bd, Cd, n_p=n_p, n_c=n_c, r_w=r_w)
            Ky, Kmpc = self.dmpc_sys.opt_cl_gains()
            Kx = Kmpc[0, :-1]
            self.K_y = Kmpc[0, -1]
            self.K_x = Kx
        else:
            dmpc_sys = ctl.mpc.System(Ad, Bd, Cd, n_p=n_p, n_c=n_c, r_w=r_w)
            self.dmpc_sys = dmpc_sys
            F, Phi = ctl.mpc.opt_matrices(dmpc_sys.A, dmpc_sys.B, dmpc_sys.C, n_p, n_c)
            R_bar = r_w * np.eye(n_c)
            M = np.linalg.inv(Phi.T @ Phi + R_bar) @ Phi.T
            self.M = M[0, :]
            Ky, Kmpc = self.dmpc_sys.opt_cl_gains()
            Kx = Kmpc[0, :-1]
            self.K_x = Kx
            self.K_y = Kmpc[0, -1]

        self.x_1 = np.array([0, 0])
        self.u_1 = 0


    def control(self, x, u, ref):

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
    
    def __init__(self, sfb_params):
        v_in = sfb_params['v_in']
        dt = sfb_params['dt']
        Am = sfb_params['A']
        Bm = sfb_params['B'] * v_in
        Cm = sfb_params['C']
        self.Am = Am; self.Bm = Bm; self.Cm = Cm; self.dt = dt; self.v_in = v_in

        # Aug model
        Aa = np.zeros((3,3))
        Ba = np.zeros((3,1))

        Aa[:2, :2] = Am
        Aa[2, :2] = Cm
        Ba[:2, 0] = Bm[:, 0]
        self.Aa = Aa; self.Ba = Ba

        # Poles
        p1 = sfb_params['p1']
        p2 = sfb_params['p2']
        p3 = sfb_params['p3']
        self.p1 = p1; self.p2 = p2; self.p3 = p3

        c_eq = np.polymul(np.polymul([1, -p1], [1, -p2]).real, [1, -p3]).real

        # Ackermann
        Mc = np.zeros((3,3))
        Mc[:, 0] = Ba[:, 0]
        Mc[:, 1] = (Aa @ Ba)[:, 0]
        Mc[:, 2] = (Aa @ Aa @ Ba)[:, 0]

        Phi_d = c_eq[0] * Aa @ Aa @ Aa + c_eq[1] * Aa @ Aa + c_eq[2] * Aa + c_eq[3] * np.eye(3)

        Kx = np.array([[0, 0, 1]]) @ np.linalg.inv(Mc) @ Phi_d
        
        self.K_x = Kx[0, :-1]
        self.K_z = Kx[0, -1]

        self.e_1 = 0
        self.zeta_1 = 0


    def control(self, x, u, ref):
        e = (ref - x[1])
        zeta = self.zeta_1 + self.dt / 2 * (e + self.e_1)
        
        u_sfb = -self.K_x @ x + self.K_z * zeta

        self.zeta_1 = zeta
        self.e_1 = e
        
        return u_sfb
