import numpy as np

def hild(H, K, n_iter=100):

    h_d = -1 / H.diagonal()
    h = H[~np.eye(H.shape[0],dtype=bool)].reshape(H.shape[0],-1)

    lm = np.zeros(H.shape[0])
    lm_p = np.zeros(H.shape[0])
    w = np.zeros(H.shape[0])
    
    k = 0

    while k < n_iter:
        lm_p[:] = lm[:]
        for i in range(0, w.shape[0]):
            lm[i] = 0
            w[i] = h_d[i] * (K[i, 0] + h[i,:i] @ lm[:i] +  h[i,i:] @ lm[i+1:])
            if w[i] > 0: lm[i] = w[i]
        k = k + 1
        if np.allclose(lm_p, lm) == True:
            break

    # print(k)
    
    return lm
