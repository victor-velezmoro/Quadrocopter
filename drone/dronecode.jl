
using Dojo
using DojoEnvironments
using LinearAlgebra

# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)

# ### Controllers
trans_mode = normalize([1; 1; 1; 1])  # Normalized to 4 elements for the 4 rotors

# Function to distribute thrust to the four rotors
function distribute_thrust(thrust)
    # For simplicity, distribute equally; in reality, this should be more complex
    return [thrust[3], thrust[3], thrust[3], thrust[3]]
end

function velocity_controller!(environment, v_des)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    # Error terms
    error = v_des .- linear_velocity
    thrust = (10 .* error - 1 .* linear_velocity .+ 5.1) # P, D, feedforward

    rpm_thrust = distribute_thrust(thrust)  # Map thrust to 4 rotors
    rpm = rpm_thrust .* 20 .* trans_mode
    set_input!(environment, rpm)
end

function position_controller!(environment, pos_des)
    state = get_state(environment)
    pos_is = state[1:3]
    v_des = pos_des .- pos_is
    velocity_controller!(environment, v_des)
end

# Define waypoints
waypoints = [[0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [1.0, 1.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 0.5]]
current_waypoint_index = 1

function controller!(environment, k)
    global current_waypoint_index

    pos_des = waypoints[current_waypoint_index]  # Current target position
    position_controller!(environment, pos_des)

    # Move to the next waypoint if close enough to the current one
    current_pos = get_state(environment)[1:3]
    if norm(current_pos .- pos_des) < 0.1
        current_waypoint_index = current_waypoint_index % length(waypoints) + 1
    end
end

# ### Simulate
initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)

# ### Visualize
vis = visualize(quadrotor_env)
render(vis)
