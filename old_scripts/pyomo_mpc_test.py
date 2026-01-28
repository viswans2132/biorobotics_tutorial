#!/usr/bin/python3
import numpy as np
import numpy.linalg as la
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import casadi as ca

dt = 0.2

def genCostGrad(u, x, N):
    uList = np.cumsum(u.reshape(N, -1), axis=0)
    X = x + uList*dt
    # # print(X)
    return 4*X.ravel()*dt + 2*u 


def genCost(u, x, N):
    X = ca.MX(2, N+1)
    uM = u.reshape((-1, N))
    X[:,0] = x
    X[:,1:] = ca.cumsum(uM, 1)*dt + x
    
    return ca.sumsqr(X) + ca.sumsqr(u)

def genCons(u, x, xo, N):
    return ca.fmax(ca.MX.zeros(1,N) , ca.sqrt(ca.sum1(((ca.cumsum(u.reshape((-1, N)), 1)*dt + x)-xo)**2)) - 0.8)


    

def mpc_func():
    N = 10
    x = np.array([5.0, 4.0])

    # print(x.reshape(N,-1))
    # X = np.array(x.reshape(N,-1)[0])
    X = np.array(x)

    opti = ca.Opti()

    uM = opti.variable(N*2)


    # opti.subject_to(genCons)

    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.tol": 1e-6}
    solver_opts = opti.solver("ipopt", opts)

    opti.subject_to(uM<=1.5)
    opti.subject_to(uM>=-1.5)



    U = []
    bounds = [(-1.0, 1.0)] * N * 2

    xo = np.array([1.0, 1.0])
    u = np.ones(2)


    # Solving the problem
    for i in range(100):
        if np.linalg.norm(x) < 0.001:
            break

        if np.linalg.norm(u[:2]) < 0.001:
            break

        cost = genCost(uM, x, N)
        opti.minimize(cost)
        opti.subject_to(genCons(uM, x, xo, N) >= 0)
        print(f'Step: {i+1}, X:{x}')

        # Solve the optimization problem
        sol = opti.solve()

        # Get the optimal control input
        u = sol.value(uM)

        x = x + u[:2] * dt
        X = np.vstack((X, x))
        U.append(u[:2])
        print(f'Step: {i+1}, U:{u[:2]}')


    U = np.array(U)

    fig, ax = plt.subplots()
    plt.plot(X[:, 0], X[:, 1])
    ax.add_patch(plt.Circle((1, 1), radius=0.5, edgecolor='r', facecolor='none'))

    plt.figure()
    plt.plot(U)

    plt.show()

if __name__ == '__main__':
    mpc_func()
