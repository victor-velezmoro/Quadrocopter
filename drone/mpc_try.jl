using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations
using OSQP
using SparseArrays

# System parameters
HORIZON = 150
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=HORIZON)
ref_position_xyz_world = [0; 0; 0]
next_waypoint = 1
state_traj = zeros(size(get_state(quadrotor_env))[1], HORIZON)
reference_traj = zeros(4, HORIZON)
plot_dict = Dict("state_traj" => state_traj, "reference_traj" => reference_traj)

# Define system matrices (assuming a linearized model)
A = I(12)  # State transition matrix
B = I(12, 4)  # Control input matrix
Q = I(12)  # State cost matrix
R = I(4)  # Control cost matrix

# Define MPC problem
function define_mpc_problem(A, B, Q, R, N)
    nx, nu = size(B)
    Q_bar = kron(I(N), Q)
    R_bar = kron(I(N), R)
    A_bar = kron(I(N), A)
    B_bar = kron(I(N), B)
    
    # State constraints
    x_min = -Inf * ones(nx * N)
    x_max = Inf * ones(nx * N)
    
    # Control constraints
    u_min = -Inf * ones(nu * N)
    u_max = Inf * ones(nu * N)
    
    return Q_bar, R_bar, A_bar, B_bar, x_min, x_max, u_min, u_max
end

# Define cost function and constraints
N = 10  # MPC horizon
Q_bar, R_bar, A_bar, B_bar, x_min, x_max, u_min, u_max = define_mpc_problem(A, B, Q, R, N)

# Define the optimization problem
function solve_mpc(x0, ref, Q_bar, R_bar, A_bar, B_bar, x_min, x_max, u_min, u_max, N)
    nx, nu = size(B_bar)[1] // N, size(B_bar)[2] // N
    P = 2 * (B_bar' * Q_bar * B_bar + R_bar)
    q = vec(B_bar' * Q_bar * (A_bar * x0 - ref))
    
    l = [x_min; u_min]
    u = [x_max; u_max]
    
    mpc_problem = OSQP.P = P, q = q, l = l, u = u
    results = OSQP.solve(mpc_problem)
    
    u_opt = results.x[1:nu]
    return u_opt
end

# Sensing and estimation
function sensing_and_estimation(environment)
    state = get_state(environment)
    position = state[1:3]  # x, y, z
    orientation = state[4:6]  # axis*angle
    linear_velocity = state[7:9]  # vx, vy, vz
    angular_velocity = state[10:12]  # ωx, ωy, ωz 
    
    return state
end

# Main controller function
function mpc_controller!(environment, k)
    global ref_position_xyz_world
    state = sensing_and_estimation(environment)
    ref = vcat(ref_position_xyz_world, zeros(9))  # Target state (position + zeros for other states)
    
    # Solve MPC problem
    u_opt = solve_mpc(state, ref, Q_bar, R_bar, A_bar, B_bar, x_min, x_max, u_min, u_max, N)
    
    # Apply control input
    set_input!(environment, u_opt)
end

# Fly through waypoints function
function fly_through_waypoints_controller!(environment, k)
    global next_waypoint
    global ref_position_xyz_world
    waypoints = [
        [1;1;0.3],
        [2;0;0.3],
        [1;-1;0.3],
        [0;0;0.3],
    ]
    if norm(get_state(environment)[1:3] - waypoints[next_waypoint]) < 1e-1
        if next_waypoint < 4
            next_waypoint += 1
        end
    end
    ref_position_xyz_world = waypoints[next_waypoint]
    mpc_controller!(environment, k)
end

# Initialization and simulation
initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, fly_through_waypoints_controller!; record=true)

vis = visualize(quadrotor_env)
render(vis)
