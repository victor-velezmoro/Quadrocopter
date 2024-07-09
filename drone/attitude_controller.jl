using Dojo
using DojoEnvironments
using LinearAlgebra
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=500)

function controller!(environment, k)
    global current_waypoint_index
    check_waypoints!(environment, k)
end

function check_waypoints!(environment, k)

    global current_waypoint_index
    global des_pos
    waypoints = [[0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [1.0, 1.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 0.5]]
    current_waypoint_index = 1
 
    current_pos = get_state(environment)[1:3]
    des_pos = waypoints[current_waypoint_index]
    
 
    if norm(current_pos .- des_pos) < 0.1 && current_waypoint_index <= length(waypoints)
        current_waypoint_index = current_waypoint_index % length(waypoints) + 1 
        println("Next waypoint: ", waypoints[current_waypoint_index])
        des_pos = waypoints[current_waypoint_index]
    end
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

    return position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index
end


function position_controller!(environment, k)
    # here we will get the current_roll, current_pitch, current_yaw and thrust commands

    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    
    #This loop translates position errors into desired roll angles.
    K_P_1 = 1.0
    K_D_1= 0.1


    des_roll = -(K_P_1 * (des_pos[2] - position[2]) + K_D_1 * (-linear_velocity[2]))
    des_ptich = K_P_1 * (des_pos[1] - position[1]) + K_D_1 * (linear_velocity[1])

    

end

function attitude_controller!(environment, k)
    # here we will calculate the controll commands using ref and current state 
    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    K_P_2 = 1.0
    K_D_2 = 0.1
    error_z = des_pos[3] - altitude
    v_z = linear_velocity[3]
    
    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)

    des_roll, des_pitch = position_controller!(environment, k)

    output_roll = K_P_2 * (des_roll - current_roll) - K_D_2 * (angular_velocity[1])
    output_pitch = K_P_2 * (des_pitch - current_pitch) - K_D_2 * (angular_velocity[2])  
    output_yaw = K_P_2 * (des_yaw - current_yaw) - K_D_2 * (angular_velocity[3])
    output_thrust = (10*error_z - 10* v_z + 5.1)*thrust_feedforward
    # K_p_thrust * (altitude_ref - altitude) + K_d_thrust * (0 - get_state(environment)[9])+ thrust_feedforward

    motor1, motor2, motor3, motor4 = MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    u = zeros(4)
    u = [motor1, motor2, motor3, motor4]
    set_input!(environment, u)  
end


function MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    motor1 = output_thrust + output_yaw + output_pitch + output_roll
    motor2 = output_thrust - output_yaw + output_pitch - output_roll
    motor3 = throutput_thrustust - output_yaw - output_pitch + output_roll
    motor4 = output_thrust + output_yaw - output_pitch - output_roll
    return [motor1, motor2, motor3, motor4]
end

initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)
