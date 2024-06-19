
using Dojo
using DojoEnvironments

quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)

#=function controller!(environment, k)
    state = get_state(environment)
    position = state[1:3] # x,y,z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    rotor_speeds = [20;30;40;50] # just for demonstration, does not make sense

    set_input!(environment, rotor_speeds) # rotor speeds are directly set
end=#
#Define PID controller gains for altitude control
Kp = 0.5
Ki = 0.01
Kd = 0.1
    
# Initialize PID control variables
global integral_error = 0.0
global previous_error = 0.0
    
# Define the controller function
function controller!(environment, k)
    state = get_state(environment)
    position = state[1:3]  # x, y, z
    orientation = state[4:6]  # axis*angle
    linear_velocity = state[7:9]  # vx, vy, vz
    angular_velocity = state[10:12]  # ωx, ωy, ωz
    
    # Desired altitude
    desired_altitude = 5.3
# Current altitude
   current_altitude = position[3]
    
# Altitude error
    error = desired_altitude - current_altitude
    
    # PID control
    global integral_error += error
    derivative_error = error - previous_error
    control_output = Kp * error + Ki * integral_error + Kd * derivative_error
    global previous_error = error
    
    # For simplicity, we are controlling only the vertical thrust
    rotor_speeds = [control_output, control_output, control_output, control_output]
    
    # Ensure rotor speeds are within reasonable limits
    rotor_speeds = clamp.(rotor_speeds, 0, 100)
    
    # Set the input to the environment
    set_input!(environment, rotor_speeds)
end


initialize!(quadrotor_env, :quadrotor; body_orientation=Dojo.RotZ(pi/2))
simulate!(quadrotor_env, controller!; record=true)
vis = visualize(quadrotor_env) # you can visualize even if the simulation fails
render(vis)