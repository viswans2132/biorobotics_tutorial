#!/usr/bin/python3
import numpy as np
import numpy.linalg as la
from scipy.optimize import minimize
import matplotlib.pyplot as plt

dt = 0.1


def genCost(u, x, N):
    uList = np.cumsum(u.reshape(N, -1), axis=0)
    X = x + uList*dt
    return np.dot(X.ravel(), X.ravel())*2 + np.dot(u, u)

def genCons(u, x, N):
    uList = np.cumsum(u.reshape(N,-1), axis=0)
    X = x + uList*dt
    return np.min(la.norm(X, axis=1) - 1.0)


def mpc_func():
    N = 10
    x = np.array([5.0, 4.0])

    X = np.array(x)

    u = np.zeros((N * 2))
    u[1] = 1
    U = []
    bounds = [(-1.0, 1.0)] * N * 2

    for i in range(150):
        if la.norm(x) < 0.1:
            break
        cons = [{'type': 'ineq', 'fun': genCons, 'args': (x, xo, N)}]

        result = minimize(genCost, u, args=(x, N), constraints=cons, bounds=bounds)
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
