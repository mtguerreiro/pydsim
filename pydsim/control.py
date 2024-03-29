import scipy
import numpy as np
import pyctl as ctl
import pyoctl.opt as octl
import pydsim.utils as pydutils
import pydsim.control as pydctl
import pydsim.observer as pydobs
import pydsim.dmpc as pyddmpc
import pydsim.qp as pydqp
import qpsolvers as qps


def set_controller_buck(buck, controller, params):

    # Control
    ctl = controller()
    
    if type(ctl) is pydctl.PI:
        t_pwm = buck.circuit.t_pwm
        print('f_pwm: {:.2f} kHz'.format(1/t_pwm/1e3))
        kp, ki = params['kp'], params['ki']
        ctl._set_params(kp, ki, t_pwm)
        ctl.u_1 = 1/9
        
    elif type(ctl) is pydctl.PID:
        t_pwm = buck.circuit.t_pwm
        ki = params['ki']
        kp = params['kp']
        kd = params['kd']
        N = params['N']
        ctl._set_params(kp, ki, kd, N, t_pwm)   
        
    elif type(ctl) is pydctl.SMPC:
        t_pwm = buck.circuit.t_pwm
        A, B, C = buck.model.A, buck.model.B, buck.model.C
        v_in = buck.signals.v_in[0]
        n_step = params['n_step']
        alpha, beta, il_max = params['alpha'], params['beta'], params['il_max']
        if 'ref' in params:
            ref = params['ref']
        else:
            ref = None
        ctl._set_params(A, B, C, t_pwm, v_in, n_step, alpha=alpha, beta=beta, il_max=il_max, ref=ref)
        
    elif type(ctl) is pydctl.DMPC:
        t_pwm = buck.circuit.t_pwm
        A, B, C = buck.model.A, buck.model.B, buck.model.C
        n_c, n_p, r_w = params['n_c'], params['n_p'], params['r_w']
        if 'ref' in params:
            ref = params['ref']
        else:
            ref = None
            
        if 'v_in' in params:
            v_in = params['v_in']
        else:
            v_in = buck.signals.v_in[0]
        ctl._set_params(A, B, C, t_pwm, v_in, n_p, n_c, r_w, ref)

    elif type(ctl) is pydctl.DMPC_C:
        t_pwm = buck.circuit.t_pwm
        A, B, C = buck.model.A, buck.model.B, buck.model.C

        n_c, n_p, r_w = params['n_c'], params['n_p'], params['r_w']

        if 'u_lim' in params:
            u_lim = params['u_lim']
        else:
            u_lim = [0, 1]

        if 'il_lim' in params:
            il_lim = params['il_lim']
        else:
            il_lim = [-100, 100]
            
        if 'n_ct' in params:
            n_ct = params['n_ct']
        else:
            n_ct = 1

        if 'solver' in params:
            solver = params['solver']
        else:
            solver = 'hild'
        
        if 'n_iter' in params:
            n_iter = params['n_iter']
        else:
            n_iter = 100

        if 'v_in' in params:
            v_in = params['v_in']
        else:
            v_in = buck.signals.v_in[0]
            
        ctl._set_params(A, B, C, t_pwm, v_in, n_p, n_c, r_w, solver=solver, n_iter=n_iter)
        ctl._set_constraints(u_lim, il_lim, n_ct)

    elif type(ctl) is pydctl.SFB or type(ctl) is pydctl.SFB_I:
        t_pwm = buck.circuit.t_pwm
        A, B, C = buck.model.A, buck.model.B, buck.model.C
        if 'v_in' in params:
            v_in = params['v_in']
        else:
            v_in = buck.signals.v_in[0]
        poles = params['poles']
        if 'obs' in params:
            obs = params['obs']
            obs_params = params['obs_params']
            obs_params['A'], obs_params['B'], obs_params['C'] = A, B * v_in, C
            obs_params['dt'] = t_pwm
        else:
            obs = None
            obs_params = None
        ctl._set_params(A, B, C, poles, v_in, t_pwm, obs, obs_params)
    

    elif type(ctl) is pydctl.LQR:
        t_pwm = buck.circuit.t_pwm
        A, B, C = buck.model.A, buck.model.B, buck.model.C
        v_in = buck.signals.v_in[0]
        Q, R, H, N = params['Q'], params['R'], params['H'], params['N']
        ctl._set_params(A, B, C, v_in, t_pwm, Q, R, H, N)
        
    else:
        d = params['u']
        ctl._set_params(d)

    return ctl


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

    
class SMPC:

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

        # Controller parameters
        self.n_p = None
        self.alpha = None
        self.beta = None
        self.il_max = None

        # Reference and index for changing reference
        self.ref = None
        self.__i = 0


    def _set_params(self, A, B, C, dt, v_in, n_p, alpha=1, beta=0, il_max=np.inf, ref=None):
        
        self.v_in = v_in
        self.Am, self.Bm, self.Cm = A, B * v_in, C

        self.dt = dt
        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B * v_in, C, 0), dt, method='zoh')
        self.Ad, self.Bd, self.Cd = Ad, Bd, Cd

        self.n_p = n_p
        self.alpha, self.beta, self.il_max = alpha, beta, il_max
        
        if ref is None:
            self.ref = None
            self.__i = None
        else:
            n_ref = ref.shape[0]
            _ref = np.zeros(n_ref + n_p)
            _ref[:n_ref] = ref
            _ref[n_ref:] = ref[-1]
            self.ref = _ref
            self.__i = 0
        

    def pred_cost(self, x, u, ref):

        x_u_1 = self.Ad @ x + self.Bd * u
        if np.abs(x_u_1[0, 0]) >= self.il_max:
            j_u_1 = np.inf
        else:
            j_u_1 = self.alpha * (ref - x_u_1[1, 0]) ** 2# + self.beta * u ** 2

        return x_u_1, j_u_1
    
    
    def opt(self, x, ref, n_p):
        
        x_u_0, j_u_0 = self.pred_cost(x, 0, ref[0])
        if n_p != 1:
            u_0_opt, j_0_opt = self.opt(x_u_0, ref[1:], n_p - 1)
            j_u_0 += j_0_opt

        x_u_1, j_u_1 = self.pred_cost(x, 1, ref[0])
        if n_p != 1:
            u_1_opt, j_1_opt = self.opt(x_u_1, ref[1:], n_p - 1)
            j_u_1 += j_1_opt

        if j_u_0 < j_u_1:
            j_opt = j_u_0
            u_opt = 0
        else:
            j_opt = j_u_1
            u_opt = 1
        
        return u_opt, j_opt


    def meas(self, signals, i, j):

        x = signals._x[i]
        ref = signals.v_ref[j]

        sigs = [x, ref]

        return sigs


    def control(self, sigs):

        x = sigs[0].reshape(-1, 1)
        ref = sigs[1]

        if self.ref is None:
            vref = np.zeros(self.n_p)
            vref[:] = ref
        else:
            i_i = self.__i
            i_f = self.__i + self.n_p
            vref = self.ref[i_i:i_f]
            self.__i += 1

        u_opt, j_opt = self.opt(x, vref, self.n_p)
        
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
        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B * v_in, C, 0), dt, method='zoh')
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
            self.K_mpc = Kmpc
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
            self.K_mpc = Kmpc

        self.x_1 = np.array([0.0, 0.0])
        self.u_1 = 0.0

        # Augmented model for DMPC
        Aa, Ba, Ca = pyddmpc.aug(Ad, Bd, Cd)
        self.Aa, self.Ba, self.Ca = Aa, Ba, Ca

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


class DMPC_C:

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
        self.Aa = None
        self.Ba = None
        self.Ca = None
        self.CI = None

        self.F = None
        self.Phi = None
        self.F_x = None
        self.Phi_x = None
        
        self.R_bar = None
        self.Rs_bar = None

        self.M = None
        self.E_j = None
        self.E_j_inv = None
        self.H_j = None
        
        self.n_p = None
        self.n_c = None
        self.r_w = None
        self.n_r = None

        self.u_lim = None
        self.il_lim = None

        self.n_iters = None

        # QP settings
        self.solver = None
        self.n_iter = None
        

        # Reference and index for changing reference
        self.ref = None
        self.__i = 0

        # Controller states
        self.u_1 = None
        self.x_1 = None


    def _set_params(self, A, B, C, dt, v_in, n_p, n_c, r_w, solver='hild', n_iter=100, ref=None):
        
        self.v_in = v_in
        self.Am, self.Bm, self.Cm = A, B * v_in, C

        # Discrete model
        self.dt = dt
        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((A, B * v_in, C, 0), dt, method='zoh')
        self.Ad, self.Bd, self.Cd = Ad, Bd, Cd
        
        # Augmented model for DMPC
        Aa, Ba, Ca = pyddmpc.aug(Ad, Bd, Cd)
        self.Aa, self.Ba, self.Ca = Aa, Ba, Ca

        # DMPC matrices        
        self.n_p, self.n_c, self.r_w = n_p, n_c, r_w
        F, Phi = pyddmpc.matrices(Aa, Ba, Ca, n_p, n_c)
        self.F, self.Phi = F, Phi

        Cx = np.zeros(Ca.shape)
        Cx[0, 0] = 1
        F_x, Phi_x = pyddmpc.matrices(Aa, Ba, Cx, n_p, n_c)
        self.F_x, self.Phi_x = F_x, Phi_x

        Rs_bar = np.ones((n_p, 1))
        self.Rs_bar = Rs_bar
        
        R_bar = r_w * np.eye(n_c)
        self.R_bar = R_bar

        E_j = Phi.T @ Phi + R_bar
        E_j_inv = np.linalg.inv(E_j)
        self.E_j, self.E_j_inv = E_j, E_j_inv

        # QP settings
        self.solver = solver
        self.n_iter = n_iter
        
        # Initial conditions
        self.x_1 = np.array([0.0, 0.0])
        self.u_1 = 0.0


    def _set_constraints(self, u_lim, il_lim, n):

        self.n_r = n

        n_p, n_c = self.n_p, self.n_c
        E_j_inv, Phi_x = self.E_j_inv, self.Phi_x
        
        C2 = np.tril(np.ones((n, n_c)))
        CI = np.tril(np.ones((n, n_p)))
        self.CI = CI
        
        
        M = np.zeros((4*n, n_c))
        n_1 = n
        n_2 = int(2 * n)
        n_3 = int(3 * n)
        
        M[:n_1, :] = -C2
        M[n_1:n_2, :] = C2

        M[n_2:n_3, :] = -CI @ Phi_x
        M[n_3:, :] = CI @ Phi_x
        
        self.M = M

        H_j = M @ E_j_inv @ M.T
        self.H_j = H_j

        y = np.zeros((M.shape[0], 1))
        self.y = y

        self.u_lim = u_lim
        self.il_lim = il_lim
        self.n_iters = []
        
        
    def meas(self, signals, i, j):

        x = signals._x[i]
        ref = signals.v_ref[j]

        sigs = [x, ref]

        return sigs

    
    def control(self, sigs):

        Phi, F, Rs_bar = self.Phi, self.F, self.Rs_bar
        E_j, E_j_inv, H_j, M, y = self.E_j, self.E_j_inv, self.H_j, self.M, self.y
        u_lim, il_lim = self.u_lim, self.il_lim
        F_x, CI = self.F_x, self.CI
        n_iter = self.n_iter
        
        x = sigs[0]
        ref = sigs[1]

        dx = x - self.x_1
        xa = np.array([dx[0], dx[1], x[1]]).reshape(-1,1)

        # print('x:\n', x)
        # print('ref:\n', ref)
        # print('xa:\n', xa)
        # print('ua_1:\n', self.u_1)
        F_j = -Phi.T @ (Rs_bar * ref - F @ xa)

        il_min = il_lim[0] - x[0]
        il_max = il_lim[1] - x[0]
        
        self.xa=xa
        n = round(M.shape[0] / 4)
        n_1 = n
        n_2 = int(2 * n)
        n_3 = int(3 * n)
        
        y[:n_1, 0] = -u_lim[0] + self.u_1
        y[n_1:n_2, 0] = u_lim[1] - self.u_1
        y[n_2:n_3, :] = -il_min + CI @ F_x @ xa
        y[n_3:, :] = il_max - CI @ F_x @ xa

        self.xa = xa

        if self.solver == 'quadprog':
            du_opt = qps.solve_qp(E_j, F_j.reshape(-1), M, y.reshape(-1))
            if du_opt is None:
                du_opt=[-self.u_1+self.u_lim[0]]
                #du_opt=[0]
                
        elif self.solver == 'gurobi':
            du_opt = qps.gurobi_solve_qp(E_j, F_j.reshape(-1), M, y.reshape(-1))
            if du_opt is None:
                du_opt=[-self.u_1+self.u_lim[0]]
                #du_opt=[0]
                
        else:
            K_j = y + M @ E_j_inv @ F_j
            
            lm, n_iters = pydqp.hild(H_j, K_j, n_iter=n_iter, ret_n_iter=True)
            self.n_iters.append(n_iters)
            lm = lm.reshape(-1, 1)
            #if n_iters == n_iter:
            #    du_opt = [-self.u_1+self.u_lim[0]]
            #    #du_opt = [0]
            #else:
            #    du_opt = -E_j_inv @ (F_j + M.T @ lm)
            #    du_opt = du_opt.reshape(-1)
            du_opt = -E_j_inv @ (F_j + M.T @ lm)
            du_opt = du_opt.reshape(-1)

        self.F_j = F_j
        # print('\n-------------')
        # print('x:\n', x)
        # print('ref:\n', ref)
        # print('x_1:\n', self.x_1)
        # print('ua_1:\n', self.u_1)
        # print('\nlambda:\n', lm)
        # print('\nn_iter:\n', n_iters)
        # print('\ndu_opt:\n', du_opt)
        # print('-------------\n')
        
        # print('\n-------------')
        # print('E_j:\n', E_j)
        # print('\nF_j:\n', F_j)
        # print('\nM:\n', M)
        # print('\ny:\n', y)
        # print('\nlambda:\n', lm)
        # print('\ndu_opt:\n', du_opt)
        # print('-------------\n')

        u_dmpc = self.u_1 + du_opt[0]
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

        # Gains
        self.Kx = None

        # Observer
        self.obs = None
        

    def _set_params(self, A, B, C, poles, v_in, dt, obs=None, obs_params=None):

        self.poles = poles
        self.v_in = v_in
        self.dt = dt

        self.A = A
        self.B = B * v_in
        self.C = C
        
        # Ackermann
        sp = scipy.signal.place_poles(A, B * v_in, poles)
        self.Kx = sp.gain_matrix[0, :]
        print('Kx:', self.Kx)

        # Set observer
        if obs is not None:
            self.obs = obs()
            self.obs._set_params(obs_params)

            
    def meas(self, signals, i, j):
        x = signals._x[i]
        r = signals.v_ref[j] / signals.v_in[0]

        sigs = [x, r]
        
        return sigs
    

    def control(self, sigs):
        x = sigs[0]
        r = sigs[1]
        y = x[1]

        if self.obs is not None:
           x = self.obs.get_current_state(y)
        
        u_sfb = -self.Kx @ x + r

        if self.obs is not None:
            self.obs.estimate(y, u_sfb)
        
        return u_sfb
    

class SFB_I:
    
    def __init__(self):

        # Controller parameters
        self.dt = None
        self.v_in = None

        # Poles
        self.poles = None

        # Plant
        self.A = None
        self.B = None
        self.C = None

        self.Aa = None
        self.Ba = None
        self.Ca = None

        # State feedback gains
        self.Kx = None
        self.Kz = None

        # Controller states
        self.e_1 = 0
        self.zeta_1 = 0

        # Observer
        self.obs = None


    def _set_params(self, A, B, C, poles, v_in, dt, obs=None, obs_params=None):

        self.A = A
        self.B = B * v_in
        self.C = C

        self.poles = poles
        self.v_in = v_in
        self.dt = dt

        self.obs = obs

        # Augmented model for state feedback with integrator
        Aa, Ba = self._aug_model(A, B * v_in, C)
        
        # State feedback gains
        sp = scipy.signal.place_poles(Aa, Ba, poles)
        self.Kx = sp.gain_matrix[0, :-1]
        self.Kz = sp.gain_matrix[0, -1]
        print('Kx:', self.Kx)

        # Set observer
        if obs is not None:
            self.obs = obs()
            self.obs._set_params(obs_params)


    def _aug_model(self, A, B, C):
        
        Aa = np.zeros((3,3))
        Ba = np.zeros((3,1))        

        Aa[:2, :2] = A
        Aa[2, :2] = C
        Ba[:2, 0] = B[:, 0]

        return Aa, Ba

                
    def meas(self, signals, i, j):
        x = signals._x[i]
        r = signals.v_ref[j] #/ signals.v_in[0]

        sigs = [x, r]
        
        return sigs
    

    def control(self, sigs):

        dt = self.dt

        x = sigs[0]
        r = sigs[1]
        y = x[1]

        # Discrete integrator
        e = (r - y)
        zeta = self.zeta_1 + dt / 2 * (e + self.e_1)

        if self.obs is not None:
           x = self.obs.get_current_state(y)
            
        # State feedback + integrator
        u_sfb = -self.Kx @ x + self.Kz * zeta
        if u_sfb > 1: u_sfb = 1
        elif u_sfb < 0: u_sfb = 0

        if self.obs is not None:
            self.obs.estimate(y, u_sfb)

        self.zeta_1 = zeta
        self.e_1 = e
                
        return u_sfb


class LinSFB:
    
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
        self.K_x = None
        self.K_z = None

        # Controlle states
        self.e_1 = 0
        self.zeta_1 = 0

    def _set_params(self, A, B, C, poles, v_in, dt):

        self.poles = poles
        self.v_in = v_in
        self.dt = dt

        self.A = A
        self.B = B
        self.C = C

        # Augmented model
        Aa, Ba = self._aug_model(A, B, C)
        
        # Ackermann
        K_x = self._acker(Aa, Ba, poles)
        self.K_x = K_x[0, :-1]
        self.K_z = K_x[0, -1]
        print('K_x:', self.K_x)
        print('K_z:', self.K_z)


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
        x = signals._x[i] - signals.x_lp
        r = signals.v_ref[j]
        u_lp = signals.u_lp

        sigs = [x, r, u_lp]
        
        return sigs
    

    def control(self, sigs):
        x = sigs[0]
        r = sigs[1]
        u_lp = sigs[2]

        e = (0 + x[1])
        zeta = self.zeta_1 + self.dt / 2 * (e + self.e_1)
        
        u_sfb = -(self.K_x @ x + self.K_z * zeta)

        self.zeta_1 = zeta
        self.e_1 = e
        
        return u_sfb + u_lp

    
class LQR:

    def __init__(self):

        # Continuous model
        self.A = None
        self.B = None
        self.C = None
        self.v_in = None

        # Augmented model
        self.Aa = None
        self.Ba = None
        self.Ca = None
        self.v_in = None
        
        # Discrete model for LQR
        self.dt = None
        self.Ad = None
        self.Bd = None
        self.Cd = None
        
        # LQR parameters
        self.Q = None
        self.R = None
        self.H = None
        self.N = None

        # LQR feedback gain
        self.F = None

        # Feedback gains
        self.K_x = None
        self.K_z = None

        # Controller states
        self.e_1 = 0
        self.zeta_1 = 0


    def _set_params(self, A, B, C, v_in, dt, Q, R, H, N):

        # Continuous model
        self.A = A
        self.B = B * v_in
        self.C = C
        self.dt = dt

        # LQR parameters
        self.Q = Q
        self.R = R
        self.H = H
        self.N = N

        # Augmented model for state feedback with integrator
        Aa, Ba, Ca = self._aug_model(A, B * v_in, C)
        self.Aa, self.Ba, self.Ca = Aa, Ba, Ca

        # Discretization of the augmented model
        Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((Aa, Ba, Ca, 0), dt, method='zoh')
        self.Ad, self.Bd, self.Cd = Ad, Bd, Cd

        F = -octl.dynprg_gains(Ad, Bd, Q, R, H, N)
        self.F = F[0]
        self.K_x = F[0, :2]
        self.K_z = F[0, -1]
        print('K_x:', self.K_x)
        print('K_z:', self.K_z)


    def _aug_model(self, A, B, C):
        
        Aa = np.zeros((3,3))
        Ba = np.zeros((3,1))
        Ca = np.zeros((1,3))

        Aa[:2, :2] = A
        Aa[2, :2] = C
        Ba[:2, 0] = B[:, 0]
        Ca[0, :2] = C

        return Aa, Ba, Ca
    

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

        u_sfb = -self.K_x @ x + self.K_z * zeta

        self.zeta_1 = zeta
        self.e_1 = e
        
        return u_sfb

    
class FBL:

    def __init__(self):

        self.R = None
        self.L = None

        self.ki = None
        

    def _set_params(self, R, L, ki):

        self.R = R
        self.L = L

        self.ki = ki


    def meas(self, signals, i, j):

        x = signals._x[i]
        r = signals.v_ref[j] #/ signals.v_in[0]
        v_in = signals.v_in[j]

        sigs = [x, r, v_in]

        return sigs

    
    def control(self, sigs):

        R = self.R
        L = self.L
        ki = self.ki
        
        x = sigs[0]
        v_ref = sigs[1]
        v_in = sigs[2]

        il = x[0]
        vc = x[1]

        i_ref = v_ref ** 2 / (R * v_in)

        e = i_ref - il

        u = 1 + L / vc * (ki * e - v_in / L)

        return u

##def set_controller(controller):
##    ctlrs = [c[1] for c in inspect.getmembers(sys.modules[__name__], inspect.isclass)]
##
##    return ctlrs
