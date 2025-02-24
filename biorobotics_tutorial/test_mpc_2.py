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
	curX = [2.0, 2.0]
	refX = [0.0, 0.0]
	obsX = [1.0, 1.0]
	safety = 0.5
	x = np.array([curX[0], curX[1], refX[0], refX[1], obsX[0], obsX[1], safety])

	# Initialize the control inputs
	u = [1.0, 1.0]*N


	mng.ping()


	# Loop for 100 time steps
	for i in range(tf):
		# Find the solution that minimizes the cost
		sol = mng.call(x, initial_guess=u, buffer_len=8*4096)

		# Store the control inputs over the horizon
		u = np.array(sol['solution'])

		# Update the current state
		x[:2] = x[:2] + u[:2]*dt

		# Store the current values for plotting
		X.append(x[:2])
		U.append(u[:2])

	# When the iteration is over, kill the optimizer
	mng.kill()

	X = np.array(X)
	U = np.array(U)


	# print(f"Solution: {sol['solution']}")
	# print(f"Progression: {X}")

	# Plot the state along with state constraints
	fig = plt.figure()
	# Correct the plot appropriately.
	plt.plot(X)
	plt.title('Position Plot')

	# Plot the control input with input constraints
	fig2 = plt.figure()
	# Correct the plot appropriately.
	# YOUR CODE HERE
	plt.plot(U)
	plt.plot([0, tf], [-0.6, -0.6], 'k:')
	plt.plot([0, tf], [0.6, 0.6], 'k:')
	plt.title('Control Plot')



	# Make a 2D plot of the states
	fig3, ax = plt.subplots()
	# Correct the plot appropriately.
	# YOUR CODE HERE
	plt.plot(X[:,0], X[:,1])
	ax.add_patch(plt.Circle((1, 1), radius=safety, edgecolor='r', facecolor='none'))
	plt.title('2D Position')

	plt.show()









if __name__ == '__main__':
    mpc_test()
