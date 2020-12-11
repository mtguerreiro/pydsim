import numba

@numba.njit()
def sim(x, A, B, u, n):
    for i in range(n):
        x[i + 1] = A @ x[i] + B @ u[i]
