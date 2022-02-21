import numpy as np
from scipy import linalg


def projection_matrix(a):
    return a @ linalg.inv(a.T @ a) @ a.T


def triangle_projection(p, tri):
    oa = tri[1] - tri[0]
    ob = tri[2] - tri[0]
    a = np.array([oa, ob])
    return np.dot(projection_matrix(a), np.array(p))


def softmax(x):
    y = np.exp(x)
    f_x = y / np.sum(np.exp(x))
    return f_x


def get_g(x):
    return x / x.sum()
