import numpy as np

def hild(H, K, M, y, n_iter=100):

    h_d = -1 / H.diagonal()
    h = H[~np.eye(H.shape[0],dtype=bool)].reshape(H.shape[0],-1)

    lm = np.zeros(M.shape[0])
    lm_p = np.zeros(M.shape[0])
    w = np.zeros(M.shape[0])
    
    k = 0

    while k < n_iter:
        lm_p[:] = lm[:]
        for i in range(0, w.shape[0]):
            lm[i] = 0
            w[i] = h_d[i] * (K[i, 0] + h[i,:i] @ lm[:i] +  h[i,i+1:] @ lm[i+2:])
            if w[i] > 0: lm[i] = w[i]
            #else: lbd[i] = 0
        k = k + 1
        #print('iter:', k)
        #print('lambda:', lm)
        #print('lambda_p:', lm_p)
        if np.allclose(lm_p, lm) == True:
            break

    print('\n-----')
    print(lm)
    print(k)
    print('-----\n')
    
    return lm
