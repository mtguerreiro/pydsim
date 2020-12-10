
def quantize(x, res, ref):

    n = int(2 * (x / ref) * (2 ** res - 1))
    if (n % 2) == 1:
        n = n + 1

    return int(n / 2)
