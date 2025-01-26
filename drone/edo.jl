using Dojo
using DojoEnvironments
using LinearAlgebra
 
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=250)

function calculate_rotor_speeds(thrust, pitch, roll, yaw)
    # Base thrust plus adjustments for pitch, roll, and yaw
    motor1 = thrust - pitch + roll + yaw
    motor2 = thrust - pitch - roll - yaw
    motor3 = thrust + pitch - roll + yaw
    motor4 = thrust + pitch + roll - yaw

    return [motor1, motor2, motor3, motor4]
end

function orientation_PD(environment, des_vz, des_pitch, des_roll, des_yaw, des_x, des_y)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    roll = orientation[1]
    pitch = orientation[2]
    yaw = orientation[3]

    K_p_pitch = 30
    K_d_pitch = 3
    K_i_pitch = 0.3

    K_p_roll = 30
    K_d_roll = 3
    K_i_roll = 0.3


    u_thrust=20*((10*(des_vz - linear_velocity[3]) - 1*linear_velocity[3] + 5.1)*(1/sqrt(4)))
    u_pitch = (K_p_pitch * (des_pitch - pitch) + K_d_pitch * (0 - angular_velocity[2])+K_i_pitch*(des_x-position[1]))*(1/sqrt(2))
    u_roll = (K_p_roll * (des_roll + roll) + K_d_roll * (0 + angular_velocity[1])+K_i_roll*(des_y-position[2]))*(1/sqrt(2))
    u_yaw = des_yaw=0

    println("e_roll",des_roll - roll)
    println("e_angular_velocity", 0 - angular_velocity[1])
    println("u_roll",u_roll)

    return calculate_rotor_speeds(u_thrust, u_pitch, u_roll, u_yaw)
end

function position_PD(environment, des_x, des_y, des_z)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    K_p_pitch = 0.065
    K_d_pitch = 0.15

    K_p_roll = 0.065
    K_d_roll = 0.15

    des_vz = des_z - position[3]

    des_pitch = (K_p_pitch * (des_x - position[1]) + K_d_pitch * (0 - linear_velocity[1]))
    des_roll = (K_p_roll * (des_y - position[2]) + K_d_roll * (0 - linear_velocity[2]))
    des_yaw = 0

    println("e_pos",des_y - position[2])
    println("e_velocity",0 - linear_velocity[2])
    println("des_roll",des_roll)

    return orientation_PD(environment, des_vz, des_pitch, des_roll, des_yaw, des_x, des_y)
end

function controller!(environment, k)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    x = 5
    y = 5
    z = 1

    rotor_speeds = position_PD(environment, x, y, z)

    set_input!(environment, rotor_speeds) # rotor speeds are directly set
end

initialize!(quadrotor_env, :quadrotor; body_orientation=Dojo.RotZ(-pi/4))
simulate!(quadrotor_env, controller!; record=true)
vis = visualize(quadrotor_env) # you can visualize even if the simulation fails
render(vis)
