#!/usr/bin/python3
import numpy as np
import numpy.linalg as la
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from casadi import *

dt = 0.1

    

def mpc_func():
    N = 10
    x = np.array([5.0, 4.0])

    xo = np.array([5.0, 5.0])    

    u = MX.sym('u', 10)
    uM = u.reshape((-1,5))
    uL = cumsum(uM,1)
    # X = MX.sym('M', 2,5)
    X = MX(2,6)
    X[:,0] = x
    X[:,1:] = x + uL*dt

    dist1 = sum1((X-xo)**2)
    dist2 = sumsqr(X)
    dist3 = sumsqr(u)

    diff = uM[:,1:] - uM[:,:-1]


    f = Function('f', [u], [uL,diff])
    
    print(f(np.arange(10)))




if __name__ == '__main__':
    mpc_func()
