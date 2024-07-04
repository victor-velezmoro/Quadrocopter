using Dojo
using DojoEnvironments
using LinearAlgebra

# ### Environment
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=2000)

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

    # Error terms
    error = v_des .- linear_velocity
    thrust = (10 .* error .- 1 .* linear_velocity .+ 5.1) # P, D, feedforward

    # Debugging prints
    println("Velocity Error: ", error)
    println("Thrust: ", thrust)
    rpm_thrust = distribute_thrust(thrust)  # Map thrust to 4 rotors
    rpm_torque = attitude_controller!(environment, ω_des, dt)  # Attitude control

    # Combine thrust and torque (simplified)
    rpm = rpm_thrust .* 20 .* trans_mode .+ rpm_torque
    println("RPM: ", rpm)
    set_input!(environment, rpm)
end
# Position controller
function position_controller!(environment, pos_des, dt)
    state = get_state(environment)
    pos_is = state[1:3]
    v_des = pos_des .- pos_is

    # Desired angular velocities (tune these based on the desired orientation)
    ω_des = [0.0, 0.0, 0.0]

    # Debugging prints
    println("Position Desired: ", pos_des)
    println("Position Error: ", v_des)

    velocity_controller!(environment, v_des, ω_des, dt)
end
# PID Attitude controller
function attitude_controller!(environment, ω_des, dt)
    global integral_error
    global previous_error

    state = get_state(environment)
    angular_velocity = state[10:12] # ωx, ωy, ωz 
     # Error terms
     error = ω_des .- angular_velocity
     integral_error += error * dt
     derivative_error = (error - previous_error) / dt
 
     torque = Kp .* error .+ Ki .* integral_error .+ Kd .* derivative_error
     previous_error = error
 
     # Debugging prints
     println("Attitude Error: ", error)
     println("Integral Error: ", integral_error)
     println("Derivative Error: ", derivative_error)
     println("Torque: ", torque)
 
     # Apply the torque to the motors
     rpm_torque = [torque[1], -torque[1], torque[2], -torque[2]]
 
     return rpm_torque
 end
 # Define waypoints
 waypoints = [[0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [1.0, 1.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 0.5]]
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
 
     # Debugging prints
     println("Current Position: ", current_pos)
     println("Current Waypoint: ", pos_des)
     println("Waypoint Index: ", current_waypoint_index)
 end
 # ### Simulate
 initialize!(quadrotor_env, :quadrotor)
 simulate!(quadrotor_env, controller!; record=true)
 
 # ### Visualize
 vis = visualize(quadrotor_env)
 render(vis)





