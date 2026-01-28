#!/usr/bin/python3
# Import standard libraries
import numpy as np
import matplotlib.pyplot as plt

# Import the symbolic variable library
import casadi as ca

# Import Opengen library
import opengen as og




def mpc_test():
    mng = og.tcp.OptimizerTcpManager('bd/quad_mpc')
    mng.start()

    safety_radius = 0.5

    x = np.array([2.1, 2.0, 0.0, 0.0, 1.0, 1.0, safety_radius])
    mng.ping()

    N = 20
    dt = 0.05
    X = []
    U = []

    for i in range(200):
    	sol = mng.call(x, initial_guess=[-1.0, -1.0]*N, buffer_len=8*4096)
    	u = np.array(sol['solution'][:2])
    	x[:2] = x[:2] + u*dt

    	X.append([x[0], x[1]])
    	U.append(u)

    mng.kill()

    X = np.array(X)
    U = np.array(U)


    # print(f"Solution: {sol['solution']}")
    # print(f"Progression: {X}")

    fig = plt.figure()
    plt.plot(X[:,0])
    plt.plot(X[:,1])

    fig2 = plt.figure()
    plt.plot(U)


    fig3, ax  = plt.subplots()
    ax.add_patch(plt.Circle((1, 1), radius=safety_radius, edgecolor='r', facecolor='none'))
    plt.plot(X[:,0], X[:,1])
    plt.axis('equal')
    plt.title('2D Position Plot')


    plt.show()









if __name__ == '__main__':
    mpc_test()
