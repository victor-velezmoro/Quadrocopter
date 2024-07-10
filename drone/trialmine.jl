using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations
# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=200, gravity=0)

# ### PID Controller Parameters
Kp = [2.0, 2.0, 2.0]   # Proportional gain
Ki = [0.1, 0.1, 0.1]   # Integral gain
Kd = [0.1, 0.1, 0.1]   # Derivative gain
trans_mode = normalize([1; 1; 1; 1]) 
# State for the integral term and previous error
integral_error = zeros(3)
previous_error = zeros(3)
# Function to distribute thrust to the four rotors
function distribute_thrust(thrust)
    return [thrust[3], thrust[3], thrust[3], thrust[3]]
end
# Velocity controller
function velocity_controller!(environment, v_des, ω_des, dt)
    state = get_state(environment)
    linear_velocity = state[7:9] # vx, vy, vz

    orientation = state[4:6] # axis*angle
    theta_is = norm(orientation)
    # Error terms
    error = v_des .- linear_velocity
    #thrust = (10 .* error .- 1 .* linear_velocity .+ 5.1) # P, D, feedforward
    theta_des=error
    error_theta=theta_des. - theta_is
    attitude_controller!(error_theta)
end
    #bekomme aus orientation, vllt package rotation
   #u1, u2, u3, u4 rotor speeds die im im quadrat thrust ergeben rpm aus rom to force torque umdrehen
   #u1 =10+

    #rpm_thrust = distribute_thrust(thrust)  # Map thrust to 4 rotors
    # rpm_torque = attitude_controller!(environment, ω_des, dt)  # Attitude control

    # # Combine thrust and torque
    # rpm = rpm_thrust .* 20 .* trans_mode .+ rpm_torque
    # set_input!(environment, rpm)

# Position controller
function position_controller!(environment, pos_des, dt)
    state = get_state(environment)
    pos_is = state[1:3]
    v_des = pos_des .- pos_is

    # Desired angular velocities 
    ω_des = [0.0, 0.0, 0.0]


    velocity_controller!(environment, v_des, ω_des, dt)
end
# PID Attitude controller
function attitude_controller!(environment, error_theta, error, dt)
    #F1 = ... theta_error
    #F2
    #F3
    #F4

    F1=0+5*error_theta[1]+5*error_theta[2]+5*error[3]
    F2=0-5*error_theta[1]+5*error_theta[2]+5*error[3]
    F3=0-5*error_theta[1]-5*error_theta[2]+5*error[3]
    F4=0-5*error_theta[1]-5*error_theta[2]+5*error[3]

    #force = sign(rpm)*force_factor*rpm^2

    #u1 = ... F1 ...
    #u2 = ... F2 ...
    #u3 = ... F3 ...
    #u4 = ... F4 ...
    force_factor = 0.001
    u1 = sign(F1) * sqrt(abs(F) / force_factor)
    u2 = sign(F2) *sqrt(abs(F2)/force_factor)
    u3= sign(F3) *sqrt(abs(F3)/force_factor)
    u4=sign(F4) *sqrt(abs(F4)/force_factor)

    #function rpm_to_force_torque(::QuadrotorWaypoint, rpm::Real, rotor_sign::Int64)
       # force_factor = 0.001
       # torque_factor = 0.0001
    
       # force = sign(rpm)*force_factor*rpm^2
        #torque = sign(rpm)*rotor_sign*torque_factor*rpm^2
    
        #return [force;0;0], [torque;0;0]
    #end

    #F=Fg/4+-Kp*(theta_error komponenten addiert)

    # global integral_error
    # global previous_error

    # state = get_state(environment)
    # angular_velocity = state[10:12] # ωx, ωy, ωz 
    #  # Error terms
    #  error = ω_des .- angular_velocity
    #  integral_error += error * dt
    #  derivative_error = (error - previous_error) / dt
 
    #  torque = Kp .* error .+ Ki .* integral_error .+ Kd .* derivative_error
    #  previous_error = error
 
    #  # Apply the torque to the motors
    #  rpm_torque = [torque[1], -torque[1], torque[2], -torque[2]]
 
    #  return rpm_torque
 end
 # Define waypoints
 waypoints = [[1.0, 1.0, 0.3], [2.0, 0, 0.3], [1.0, -1.0, 0.3], [0.0, 0.0, 0.3], [1.0, 1.0, 0.3]]
 current_waypoint_index = 1
 
 # Main controller
 function controller!(environment, k)
     global current_waypoint_index
 
     dt = 0.1 # Time step (example value, should be based on actual simulation time step)
 
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





