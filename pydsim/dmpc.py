import numpy as np
import pydsim.dmpc as dmpc


def aug(Am, Bm, Cm):
    r"""Determines the augmented model. For now, only one control signal and
    one output is supported.

    Parameters
    ----------
    Am : np.array
        An (n, n) numpy matrix.

    Bm : np.array
        An (n, 1) numpy matrix.

    Cm : np.array
        A (1, n) numpy matrix.

    Returns
    -------
    (A, B, C) : tuple
        A tuple containing the augmented matrices.
    
    """
    n = Am.shape[0]
    zeros_n = np.zeros((n, 1))

    A = np.zeros((n + 1, n + 1))
    A[:n, :n] = Am
    A[-1, :n] = Cm @ Am
    A[-1, -1] = 1

    B = np.zeros((n + 1, 1))
    B[:n] = Bm
    B[-1] = Cm @ Bm

    C = np.zeros((1, n + 1))
    C[0, -1] = 1

    return (A, B, C)


def opt(A, B, C, x_i, r, r_w, n_p, n_c):
    r"""Provides the control vector miniming the expression

    .. :math:

        J = (R_s - Y)^T(R_s - Y) + \Delta U^T R \Delta U.

    Parameters
    ----------
    A : :class:`np.array`
        The `A` matrix of the augmented model. An (n, n) numpy matrix.

    B : :class:`np.array`
        The `B` matrix of the augmented model. An (n, 1) numpy matrix.

    C : :class:`np.array`
        The `B` matrix of the augmented model. An (1, n) numpy matrix.

    x_i : :class:`np.array`
        Initial conditions of the augmented states. An (n, 1) numpy matrix.

    r : :class:`int`, :class:`float`
        The set-point signal.

    r_w : :class:`int`, :class:`float`
        Weight of the control action.

    n_p : :class:`int`
        Length of prediction horizon.

    n_c : :class:`int`
        Length of the control window.

    Returns
    -------
    :class:`np.array`
        An (n_c, 1) numpy matrix containing the optimal control values.

    """
    x_i = x_i.reshape(-1, 1)
    R_s = r * np.ones((n_p, 1))
    R = r_w * np.eye(n_c)

    F = np.zeros((n_p, (C @ A).shape[1]))
    F[0, :] = C @ A
    for i in range(1, n_p):
        F[i, :] = F[i - 1, :] @ A
    
    Phi = np.zeros((n_p, n_c))
    Phi[0, 0] = C @ B
    for i in range(1, n_p):
        A_p = np.linalg.matrix_power(A, i)
        Phi[i, 0] = C @ A_p @ B
        for j in range(n_c - 1):
            Phi[i, j + 1] = Phi[i - 1, j]

    Phi_t = Phi.T

    DU = np.linalg.inv(Phi_t @ Phi + R) @ Phi_t @ (R_s - F @ x_i)
    
    return DU


def predict_horizon(A, B, C, u, x_i, n_p):
    r"""Predicts the system's response for a given control action and a given
    horizon.

    Parameters
    ----------
    A : :class:`np.array`
        The `A` matrix of the augmented model. An (n, n) numpy matrix.

    B : :class:`np.array`
        The `B` matrix of the augmented model. An (n, 1) numpy matrix.

    C : :class:`np.array`
        The `B` matrix of the augmented model. An (1, n) numpy matrix.

    u : :class:`np.array`
        The control values. An (n_c, 1) numpy matrix, where `n_c` is the
        number of control actions.

    x_i : :class:`np.array`
        Initial conditions of the augmented states. An (n, 1) numpy matrix.

    n_p : :class:`int`
        Length of prediction horizon. Should be equal or greater than the
        number of control actions.
    
    Returns
    -------
    (x, y) : :class:`tuple`
        A tuple containing two numpy matrices. The first matrix contains the
        state values `x` and the second matrix contains the output `y`.
    
    """
    x_i = x_i.reshape(-1, 1)
    n_c = u.shape[0]

    x = np.zeros((n_p, x_i.shape[0]))
    y = np.zeros((n_p, C.shape[0]))

    x[0, :] = x_i.reshape(-1)
    y[0, :] = C @ x[0, :]
    for i in range(1, n_c):
        x[i, :] = A @ x[i - 1, :] + B @ u[i - 1]
        y[i, :] = C @ x[i, :]
        
    for i in range(n_c, n_p):
        x[i, :] = A @ x[i - 1, :]
        y[i, :] = C @ x[i, :]

    return (x, y)


def matrices(A, B, C, n_p, n_c):
    r"""Computes the :math:`F` and :math:`Phi` matrices.

    Parameters
    ----------
    A : :class:`np.array`
        The `A` matrix of the augmented model. An (n, n) numpy matrix.

    B : :class:`np.array`
        The `B` matrix of the augmented model. An (n, 1) numpy matrix.

    C : :class:`np.array`
        The `B` matrix of the augmented model. An (1, n) numpy matrix.

    n_p : :class:`int`
        Length of prediction horizon.

    n_c : :class:`int`
        Length of the control window.

    Returns
    -------
    (F, Phi) : :class:`tuple`
        A tuple, where the first item corresponds to the `F` matrix and the
        second item corresponds to the `Phi` matrix.
    
    """
    F = np.zeros((n_p, (C @ A).shape[1]))
    F[0, :] = C @ A
    for i in range(1, n_p):
        F[i, :] = F[i - 1, :] @ A

    Phi = np.zeros((n_p, n_c))
    Phi[0, 0] = C @ B
    for i in range(1, n_p):
        A_p = np.linalg.matrix_power(A, i)
        Phi[i, 0] = C @ A_p @ B
        for j in range(n_c - 1):
            Phi[i, j + 1] = Phi[i - 1, j]

    return (F, Phi)


def cl_gains(A, B, C, r_w, n_p, n_c):
    r"""Computes the optimal gains :math:`K_y` and :math:`K_{mpc}`.

    Parameters
    ----------
    A : :class:`np.array`
        The `A` matrix of the augmented model. An (n, n) numpy matrix.

    B : :class:`np.array`
        The `B` matrix of the augmented model. An (n, 1) numpy matrix.

    C : :class:`np.array`
        The `B` matrix of the augmented model. An (1, n) numpy matrix.

    r_w : :class:`int`, :class:`float`
        Weight of the control action.

    n_p : :class:`int`
        Length of prediction horizon.

    n_c : :class:`int`
        Length of the control window.
    
    Returns
    -------
    (K_y, K_mpc) : :class:`tuple`
        A tuple, containing two elements. The first element is the matrix
        K_y and the second element is the matrix K_mpc.

    """
    
    R_s_bar = np.ones((n_p, 1))
    R = r_w * np.eye(n_c)
    
    F, Phi = dmpc.matrices(A, B, C, n_p, n_c)
    Phi_t = Phi.T

    K = np.linalg.inv(Phi_t @ Phi + R) @ Phi_t
    K_mpc = K @ F
    K_y = K @ R_s_bar

    return (K_y[0].reshape(1, -1), K_mpc[0, :].reshape(1, -1))
