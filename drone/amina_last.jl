using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations

# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=300)
current_waypoint_index = 0
des_pos = [0;0;0]

function position_controller!(environment, pos_des, dt)
    state = get_state(environment)
    pos_is = state[1:3]
    v_des = pos_des .- pos_is

    # Desired angular velocities 
    ω_des = [0.0, 0.0, 0.0]

    velocity_controller!(environment, v_des, ω_des, dt)
end

# Velocity controller
function velocity_controller!(environment, v_des, ω_des, dt)
    state = get_state(environment)
    linear_velocity = state[7:9] # vx, vy, vz
    orientation = state[4:6] # axis*angle
    theta_is = norm(orientation)
    # Error terms
    error = v_des .- linear_velocity
    theta_des = error
    error_theta = theta_des .- theta_is
    #error_theta1 = roll 
    #error_theta2 = pitch
    #error_theta3 = yaw
    attitude_controller!(environment, error_theta, error, dt)
end

# PID Attitude controller
function attitude_controller!(environment, error_theta, error, dt)
    state = get_state(environment)
    altitude = state[3]
    z_errror = des_pos[3] - altitude
    v_z = state[9]
    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)

    thrust_output = (10*z_errror - 10* v_z )+ thrust_feedforward
    F1 = thrust_output+ 5 * error_theta[1] + 5 * error_theta[2] + 5 * error[3]
    F2 = thrust_output - 5 * error_theta[1] + 5 * error_theta[2] + 5 * error[3]
    F3 = thrust_output - 5 * error_theta[1] - 5 * error_theta[2] + 5 * error[3]
    F4 = thrust_output - 5 * error_theta[1] - 5 * error_theta[2] + 5 * error[3]

    # force = sign(rpm)*force_factor*rpm^2
    force_factor = 0.001
    u = zeros(4)
    u[1] =  sign(F1)*sqrt(abs(F1) / force_factor)
    u[2] =  sign(F2)*sqrt(abs(F2) / force_factor)
    u[3] = sign(F3)*sqrt(abs(F3) / force_factor)
    u[4] =  sign(F4)*sqrt(abs(F4) / force_factor)

    set_input!(environment, u)  


    #function rpm_to_force_torque(::QuadrotorWaypoint, rpm::Real, rotor_sign::Int64)
       # force_factor = 0.001
       # torque_factor = 0.0001
    
       # force = sign(rpm)*force_factor*rpm^2
        #torque = sign(rpm)*rotor_sign*torque_factor*rpm^2
    
        #return [force;0;0], [torque;0;0]
    #end
    
end

# Define waypoints
waypoints = [[1.0, 1.0, 0.3], [2.0, 0, 0.3], [1.0, -1.0, 0.3], [0.0, 0.0, 0.3], [1.0, 1.0, 0.3]]
current_waypoint_index = 1

# Main controller
function controller!(environment, k)
    global current_waypoint_index
    global des_pos

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
