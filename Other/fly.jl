using Dojo
using DojoEnvironments
using LinearAlgebra

# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)

# Define waypoints
const WAYPOINTS = [
    [1; 1; 0.3],
    [2; 0; 0.3],
    [1; -1; 0.3],
    [0; 0; 0.3],
]

# Index to keep track of current waypoint
const waypoint_idx = Ref(1)

# ### Controllers
trans_mode = normalize([1; 1; 1; 1])

function velocity_controller!(environment, v_des)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    error_v = v_des - linear_velocity
    thrust = (10 .* error_v .- 1 .* linear_velocity .+ 5.1) .* trans_mode
 # P, D, feedforward

    rpm = thrust * 20
    set_input!(environment, rpm)
end

function position_controller!(environment, pos_des)
    pos_is = get_state(environment)[1:3] # x, y, z
    v_des = pos_des .- pos_is
    velocity_controller!(environment, v_des)
end

function controller!(environment, k)
    # Get the current waypoint
    pos_des = WAYPOINTS[waypoint_idx[]]

    # Move towards the current waypoint
    position_controller!(environment, 0.8)

    # Check if the quadrotor has reached the current waypoint
    pos_is = get_state(environment)[1:3]
    if norm(pos_des - pos_is) < 0.1 # Tolerance for reaching the waypoint
        waypoint_idx[] += 1
        if waypoint_idx[] > length(WAYPOINTS)
            waypoint_idx[] = 1 # Reset to the first waypoint or handle as needed
        end
    end
end

# ### Simulate
initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)

# ### Visualize
vis = visualize(quadrotor_env)
render(vis)
