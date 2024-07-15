using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations

# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)

position_integral = zeros(3)
position_previous_error = zeros(3)
velocity_integral = zeros(3)
velocity_previous_error = zeros(3)

Kp_pos = 0.6
Ki_pos = 3.5
Kd_pos = 0.03

function position_controller!(environment, pos_des, dt)
    global position_integral, position_previous_error
    state = get_state(environment)
    pos_is = state[1:3]
    #v_des = pos_des .- pos_is
    error = pos_des .- pos_is

    # PID terms for position
    position_integral .= (position_integral .+ error) * dt
    derivative = (error .- position_previous_error) / dt
    position_previous_error .= error

    # Desired angular velocities 
    ω_des = [0.0, 0.0, 0.0]

    v_des = Kp_pos .* error .+ Ki_pos .* position_integral .+ Kd_pos .* derivative

    velocity_controller!(environment, v_des, ω_des, dt)
end  

Kp_vel = 1.0
Ki_vel = 0.1
Kd_vel = 0.2

# Velocity controller
function velocity_controller!(environment, v_des, ω_des, dt)
    global position_integral, position_previous_error
    state = get_state(environment)
    linear_velocity = state[7:9] # vx, vy, vz
    orientation = state[4:6] # axis*angle
    theta_is = norm(orientation)

    # Error terms
    error = v_des .- linear_velocity
    
    # PID terms for velocity
    velocity_integral .= (velocity_integral .+ error) * dt
    derivative = (error .- velocity_previous_error) / dt
    velocity_previous_error .= error

    theta_des = Kp_vel .* error .+ Ki_vel .* velocity_integral .+ Kd_vel .* derivative
    
    error_theta = theta_des .- theta_is

    attitude_controller!(environment, error_theta, error, dt)
end

# PID Attitude controller
function attitude_controller!(environment, error_theta, error, dt)

    F1 = 20 + 0.6* error_theta[1] -0.6 * error_theta[2] + 2 * error[3]
    F2 = 20 + 0.6 * error_theta[1] + 0.6 * error_theta[2] - 2 * error[3]
    F3 = 20 - 0.6 * error_theta[1] + 0.6 * error_theta[2] + 2 * error[3]
    F4 = 20 - 0.6 * error_theta[1] - 0.6 * error_theta[2] - 2 * error[3]

    # force = sign(rpm)*force_factor*rpm^2
    force_factor = 0.001
    u = zeros(4)
    u[1] = sign(F1) * sqrt(abs(F1) / force_factor)
    u[2] = sign(F2) * sqrt(abs(F2) / force_factor)
    u[3] = sign(F3) * sqrt(abs(F3) / force_factor)
    u[4] = sign(F4) * sqrt(abs(F4) / force_factor)

    set_input!(environment, u)  
end

# Define waypoints
waypoints = [[1.0, 1.0, 0.3], [2.0, 0, 0.3], [1.0, -1.0, 0.3], [0.0, 0.0, 0.3], [1.0, 1.0, 0.3]]
current_waypoint_index = 1

# Main controller
function controller!(environment, k)
    global current_waypoint_index

    dt = 0.1 # Time step 

    pos_des = waypoints[current_waypoint_index]  # Current target position
    position_controller!(environment, pos_des, dt)

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
