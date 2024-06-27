# Assuming DojoEnvironments package is imported and necessary setup is done

# Function to get the green point (waypoint) position from the environment
function get_waypoint_position(environment)
    # Placeholder function, replace with actual method or key
    return [0.0, 0.0, 5.0]  # Example: [x, y, z] position of the waypoint
end

# ### Controllers
trans_z_mode = normalize([1; 1; 1; 1])

function velocity_controller!(environment, v_des)
    state = get_state(environment)
    v_z_is = state[9]
    error_z = v_des - v_z_is
    thrust = (10 * error_z - 1 * v_z_is + 5.1) * trans_z_mode  # P, D, feedforward

    rpm = thrust * 20
    set_input!(environment, rpm)
end

function position_controller!(environment, z_des)
    state = get_state(environment)
    z_is = state[3]
    v_des = z_des - z_is
    velocity_controller!(environment, v_des)
end

function controller!(environment, k)
    green_point = get_waypoint_position(environment)
    z_des = green_point[3]  # Assuming green_point provides [x, y, z]

    # Control to the green point's z position
    position_controller!(environment, z_des)
end

# ### Simulation Setup
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)
initialize!(quadrotor_env, :quadrotor)

# ### Simulation
simulate!(quadrotor_env, controller!; record=true)

# ### Visualization
vis = visualize(quadrotor_env)
render(vis)
