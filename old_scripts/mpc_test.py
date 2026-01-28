#!/usr/bin/python3
import numpy as np
import numpy.linalg as la
from scipy.optimize import minimize
import matplotlib.pyplot as plt

dt = 0.1

def genCostGrad(u, x, N):
    uList = np.cumsum(u.reshape(N, -1), axis=0)
    X = x + uList*dt
    # # print(X)
    return 4*X.ravel()*dt + 2*u 


def genCost(u, x, N):
    uList = np.cumsum(u.reshape(N, -1), axis=0)
    X = x + uList*dt
    return np.dot(X.ravel(), X.ravel())*2 + np.dot(u, u)

def genCons(u, x, xo, N):
    uList = np.cumsum(u.reshape(N,-1), axis=0)
    X = x + uList*dt
    return np.min(la.norm(X - xo, axis=1) - 1.0)


def mpc_func():
    N = 10
    x = np.array([2.1, 2.0])

    # print(x.reshape(N,-1))
    # X = np.array(x.reshape(N,-1)[0])
    X = np.array(x)

    u = np.zeros((N * 2))
    u[1] = 1
    # u = np.cumsum(u.reshape(N,-1), axis=0)
    # print(u)
    U = []
    bounds = [(-1.0, 1.0)] * N * 2

    xo = np.array([1.0, 1.0])

    for i in range(150):
        if la.norm(x) < 0.1:
            break
        cons = [{'type': 'ineq', 'fun': genCons, 'args': (x, xo, N)}]

        result = minimize(genCost, u, args=(x, N), constraints=cons, bounds=bounds, jac=genCostGrad)
        u = result.x
        x = x + u[:2] * dt
        X = np.vstack((X, x))
        U.append(u[:2])
        print(f'Step: {i+1}, X:{x}')


    U = np.array(U)

    fig, ax = plt.subplots()
    plt.plot(X[:, 0], X[:, 1])
    ax.add_patch(plt.Circle((1, 1), radius=1, edgecolor='r', facecolor='none'))

    plt.figure()
    plt.plot(U)

    plt.show()

if __name__ == '__main__':
    mpc_func()
