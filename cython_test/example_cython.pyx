def test(int x):
    cdef double y = 0
    cdef int i
    for i in range(x):
        y += i
    return y