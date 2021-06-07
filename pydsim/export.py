import numpy as np


def buck_dmpc_export(buck):

    n_p, n_c, n_r = buck.ctl.n_p, buck.ctl.n_c, buck.ctl.n_r
    u_lim, il_lim = buck.ctl.u_lim, buck.ctl.il_lim

    Fj1 = -buck.ctl.Phi.T @ buck.ctl.Rs_bar
    Fj2 = buck.ctl.Phi.T @ buck.ctl.F

    Kj1 = buck.ctl.M @ buck.ctl.E_j_inv

    Fxp = buck.ctl.CI @ buck.ctl.F_x

    Hj = buck.ctl.H_j

    DU1 = (-buck.ctl.E_j_inv)[0, :]
    DU2 = (-buck.ctl.E_j_inv @ buck.ctl.M.T)[0, :]

    text = ''

    header = '/**\n'\
     '* @file dmpc_buck_matrices.h\n'\
     '* @brief Header with relevant data to run the DMPC algorithm for a buck converter.\n'\
     '*\n'\
     '* This file is generated automatically and should not be modified.\n'\
     '*\n'\
     '*  Originally created on: 27.05.2021\n'\
     '*      Author: mguerreiro\n'\
     '*/\n'
    
    text = text + header

    def_guard = '\n#ifndef DMPC_BUCK_MATRICES_H_\n'\
                '#define DMPC_BUCK_MATRICES_H_\n'
    text = text + def_guard

    defines = '\n/* Prediction, control and restriction horizon */\n'\
              '#define DMPC_BUCK_CONFIG_NP\t{:}\n'.format(n_p)+\
              '#define DMPC_BUCK_CONFIG_NC\t{:}\n'.format(n_c)+\
              '#define DMPC_BUCK_CONFIG_NR\t{:}\n'.format(n_r)
    text = text + defines


    constraints = '\n/* Constraints */\n'\
                  '#define DMPC_BUCK_CONFIG_IL_MIN\t{:}\n'.format(il_lim[0])+\
                  '#define DMPC_BUCK_CONFIG_IL_MAX\t{:}\n'.format(il_lim[1])+\
                  '#define DMPC_BUCK_CONFIG_U_MIN\t{:}\n'.format(u_lim[0])+\
                  '#define DMPC_BUCK_CONFIG_U_MAX\t{:}\n'.format(u_lim[1])
    text = text + constraints

    matrices = '\n /* Matrices */\n'
    fj1 = np_array_to_c(Fj1, 'float Fj_1') + '\n\n'
    fj2 = np_array_to_c(Fj2, 'float Fj_2') + '\n\n'
    fxp = np_array_to_c(Fxp, 'float Fx') + '\n\n'
    kj1 = np_array_to_c(Kj1, 'float Kj_1') + '\n\n'
    hj = np_array_to_c(Hj, 'float Hj') + '\n\n'
    du1 = np_array_to_c(DU1, 'float DU_1') + '\n\n'
    du2 = np_array_to_c(DU2, 'float DU_2') + '\n\n'
    text = text + matrices + fj1 + fj2 + fxp + kj1 + hj + du1 + du2

    def_guard_end = '\n#endif /* DMPC_BUCK_MATRICES_H_ */\n'
    text = text + def_guard_end

    print(text)
    
def np_array_to_c(arr, arr_name):

    if arr.ndim == 1:
        n = arr.shape[0]
        m = 1
    else:
        if (arr.shape[0] == 1) or (arr.shape[1] == 1):
            arr = arr.flatten()
            n = arr.shape[0]
            m = 1
        else:
            n, m = arr.shape

    arr_str = np.array2string(arr, separator=',')
    arr_str = arr_str.replace('[', '{')
    arr_str = arr_str.replace(']', '}')

    if m == 1:
        arr_str = '{:}[{:}] = {:};'.format(arr_name, n, arr_str)
    else:
        arr_str = '{:}[{:}][{:}] = {:};'.format(arr_name, n, m, arr_str)

    return arr_str

