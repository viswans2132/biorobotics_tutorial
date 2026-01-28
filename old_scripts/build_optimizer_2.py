#!/usr/bin/python3
import numpy as np # Numerical computation library
import casadi as ca # Symbolic variable library
import opengen as og # Opengen framework


# Function to build the optimizer
def build_mpc():
    '''
    This function builds the optimizer for MPC.
    The state variable x has current position, reference position and state limits.
    The input variable u is augmented with the future values. 
    '''

    N = 20 # Prediction horizon
    dt = 0.05 # Sampling time

    # Define symbolic variables for states and inputs
    x = ca.MX.sym('x', 7) 
    u = ca.MX.sym('u', N)

    x_cur = x[:2] # Current Position
    x_ref = x[2:4] # Reference Position
    x_obs = x[4:6] # Obstacle Position
    safety_rad = x[6] # Safety Radius

    # Controller Gains
    Q = 10
    R = 1

    # Predict the future states considering x(k+1) = x(k) + u(k)*dt:
    X = ca.cumsum(u)*dt + x_cur

    # Calculate the cost
    # YOUR CODE BELOW
    cost = 0

    # Add the state constraints ||x - x_ref|| > 1.0
    # YOUR CODE BELOW
    c = 0

    # Add the upper and lower limit of the control inputs:
    umin = [-0.6, -0.6]*N
    umax = [0.6, 0.6]*N
    bounds = og.constraints.Rectangle(umin, umax)

    # Add the control variable, states, cost, constraints and bounds to the problem.
    problem = og.builder.Problem(u, x, cost).with_constraints(bounds).with_penalty_constraints(c)
    tcp_config = og.config.TcpServerConfiguration(bind_port=3320)

    # Choose the folder where the optimizer must be built and stored.
    build_config = og.config.BuildConfiguration().with_build_directory("bd").with_build_mode("release").with_tcp_interface_config(tcp_config)
    meta = og.config.OptimizerMeta().with_optimizer_name("quad_mpc")

    # Choose solver parameters.
    solver_config = og.config.SolverConfiguration() \
                    .with_tolerance(1e-5) \
                    .with_initial_tolerance(1e-5) \
                    .with_max_duration_micros(40000) \
                    .with_max_outer_iterations(5) \
                    .with_penalty_weight_update_factor(5) \
                    .with_initial_penalty(1000.0)

    # Choose builder parameters.
    builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config).with_verbosity_level(1)

    # Build the optimizer.
    builder.build()


def mpc_test():
    '''
    This function is used to test the optimizer.
    '''

    # Create a TCP manager to invoke the optimizer.
    mng = og.tcp.OptimizerTcpManager('bd/quad_mpc')

    # Start the optimizer.
    mng.start()

    # Give your test values here.
    # YOUR CODE BELOW
    x = 0
    mng.ping()

    # Solve the problem.
    sol = mng.call(x, initial_guess=[-1.0, -1.0]*20, buffer_len=8*4096)

    # Get the control inputs.
    uM = np.array(sol['solution']).reshape((-1,20))
    print(f"Solution: {sol['solution']}")

    # Predict the states.
    X = np.cumsum(uM)*0.1 + x[:2]
    print(f"Progression: {X}")

    # Kill the optimizer.
    mng.kill()




if __name__ == '__main__':
    build_mpc()
    mpc_test()
