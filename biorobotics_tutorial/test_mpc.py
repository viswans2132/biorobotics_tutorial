#!/usr/bin/python3
# Import standard libraries
import numpy as np
import matplotlib.pyplot as plt

# Import the symbolic variable library
import casadi as ca

# Import Opengen library
import opengen as og




def mpc_test():
	'''
	This function runs an MPC by invoking the Opengen optimizer that we built.
	'''

	# Interface with the Optimizer.
	mng = og.tcp.OptimizerTcpManager('bd/quad_mpc')
	mng.start()

	N = 20 # Prediction Horizon
	dt = 0.05 # Sampling Time
	tf = 1000 # Final Time
	X = [] # Empty array for plotting
	U = [] # Empty array for plotting

	# Define the states.
	curX = 2.0
	refX = 10.0
	minX = -1.0
	maxX = 8.0
	x = np.array([curX, refX, minX, maxX])

	# Initialize the control inputs
	u = [1.0]*N


	mng.ping()


	# Loop for 1000 time steps
	for i in range(tf):
		# Find the solution that minimizes the cost
		sol = mng.call(x, initial_guess=u, buffer_len=8*4096)

		# Store the control inputs over the horizon
		u = np.array(sol['solution'])

		# Update the current state
		x[0] = x[0] + u[0]*dt

		# Store the current values for plotting
		X.append(x[0])
		U.append(u[0])

	# When the iteration is over, kill the optimizer
	mng.kill()

	X = np.array(X)
	U = np.array(U)


	# print(f"Solution: {sol['solution']}")
	# print(f"Progression: {X}")

	# Plot the state along with state constraints
	fig = plt.figure()
	plt.plot(X)
	plt.title('Position plot')
	plt.plot([0, tf], [minX, minX], 'k:')
	plt.plot([0, tf], [maxX, maxX], 'k:')

	# Plot the control input with input constraints
	fig2 = plt.figure()
	plt.plot(U)
	plt.plot([0, tf], [-0.6, -0.6], 'k:')
	plt.plot([0, tf], [0.6, 0.6], 'k:')
	plt.title('Control plot')

	plt.show()









if __name__ == '__main__':
    mpc_test()
