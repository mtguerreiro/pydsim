
def quantize(x, res, ref):

    n = int(2 * (x / ref) * (2 ** res - 1))
    if (n % 2) == 1:
        n = n + 1

    return int(n / 2)

def np_array_to_c(arr):

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
        arr_str = 'F[{:}] = {:};'.format(n, arr_str)
    else:
        arr_str = 'F[{:}][{:}] = {:};'.format(n, m, arr_str)

    return arr_str
