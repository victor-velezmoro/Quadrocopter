using Dojo
using DojoEnvironments
using LinearAlgebra
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=250, gravity = 0)
current_waypoint_index = 0
des_pos = [0;0;0]

function controller!(environment, k)
    global current_waypoint_index
    check_waypoints!(environment, k)
end

function check_waypoints!(environment, k)

    global current_waypoint_index
    global des_pos
    waypoints = [[1.0, 1.0, 0.5], [2.0, -1.0, 0.5], [1.0, -1.0, 0.5], [0.0, 1.0, 0.5]]
 
    current_pos = get_state(environment)[1:3]
    if norm(current_pos - des_pos) < 1e-1 && current_waypoint_index < length(waypoints)
        current_waypoint_index = current_waypoint_index % length(waypoints) + 1 
        #current_waypoint_index = 2

        println("Next waypoint: ", waypoints[current_waypoint_index])
        # des_pos = waypoints[current_waypoint_index]
    end
    des_pos = waypoints[current_waypoint_index]
   
    println("Des_pos: ", des_pos)
    println("current_pos: ", current_pos)
    println("current_waypoint_index: ", current_waypoint_index)

    attitude_controller!(environment, k)

end



function state_provider!(environment)
    global des_pos
    global current_waypoint_index

    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    current_roll = orientation[1]
    current_pitch = orientation[2]
    current_yaw = orientation[3]

    altitude = position[3]

    v_des = des_pos .- position
    theta_is = norm(orientation)
    error = v_des .- linear_velocity
    

    return position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index, v_des
end


function position_controller!(environment, k)
    # here we will get the current_roll, current_pitch, current_yaw and thrust commands

    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    

    # Desired angular velocities 
    ω_des = [0.0, 0.0, 0.0]

    velocity_controller!(environment, v_des, ω_des, dt)


# Velocity controller
function velocity_controller!(environment, v_des, ω_des, dt)
    state = get_state(environment)
    linear_velocity = state[7:9] # vx, vy, vz
    orientation = state[4:6] # axis*angle
    theta_is = norm(orientation)
    # Error terms
    error = v_des .- linear_velocity  #PD missing what to use for D
    theta_des = error
    error_theta = theta_des .- theta_is
    #error_theta1 = roll 
    #error_theta2 = pitch
    #error_theta3 = yaw
    attitude_controller!(environment, error_theta, error, dt)
    
    
    #This loop translates position errors into desired roll angles.
    K_P_1 = 0.04
    K_D_1= 0.04
    println("k ", k)
    println("Des_pos: ", des_pos)
    println("current_pos: ", position)
    #


    des_roll = (K_P_1 * (des_pos[1]-position[1]) + K_D_1 * (0 - linear_velocity[1]))
    des_pitch = -(K_P_1 * (des_pos[2]-position[2]) + K_D_1 * (0 - linear_velocity[2]))

    # print("des_roll: ", des_roll, " des_pitch: ", des_pitch)

    return des_roll, des_pitch

end

function attitude_controller!(environment, k)
    # here we will calculate the controll commands using ref and current state 
    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    
    


function MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    # u = zeros(4)
    
    # u[1] = output_thrust - output_yaw - output_pitch - output_roll 
    # u[2] = output_thrust - output_yaw + output_pitch - output_roll 
    # u[3] = output_thrust + output_yaw - output_pitch + output_roll 
    # u[4] = output_thrust - output_yaw + output_pitch + output_roll 



    return u
end

initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)

### Visualize
vis = visualize(quadrotor_env)
render(vis)
