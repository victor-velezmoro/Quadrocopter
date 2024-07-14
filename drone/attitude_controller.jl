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
    # waypoints = 
    # [[1,1;0.3],
    #     [2;0;0.3],
    #     [1;-1;0.3],
    #     [0;0;0.3],
    # ]

 
    current_pos = get_state(environment)[1:3]
    if norm(current_pos - des_pos) < 1e-1 && current_waypoint_index < length(waypoints)
        current_waypoint_index = current_waypoint_index % length(waypoints) + 1 
        #current_waypoint_index = 2

        println("Next waypoint: ", waypoints[current_waypoint_index])
        # des_pos = waypoints[current_waypoint_index]
    end

    # if norm(get_state(environment)[1:3]-waypoints[current_waypoint_index]) < 1e-1
    #     if current_waypoint_index < 4
    #         println("next_waypoint: ", current_waypoint_index)
    #         current_waypoint_index += 1
    #     end
    # end
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

    return position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index
end


function position_controller!(environment, k)
    # here we will get the current_roll, current_pitch, current_yaw and thrust commands

    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    
    #This loop translates position errors into desired roll angles.
    K_P_1 = 0.04
    K_D_1= 0.04
    println("k ", k)
    println("Des_pos: ", des_pos)
    println("current_pos: ", position)
    # des_roll = -(K_P_1 * (des_pos[2] - position[2]) + K_D_1 * (0 - linear_velocity[2]))
    # des_pitch = K_P_1 * (des_pos[1] - position[1]) + K_D_1 * (0 - linear_velocity[1])


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

    K_P_2 = 4.0
    K_D_2 = 1
    error_z = des_pos[3] - altitude
    v_z = linear_velocity[3]
    des_yaw = 0.0
    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)

    des_roll, des_pitch = position_controller!(environment, k)
    println("des_roll: ", des_roll, "current roll: ", current_roll, " des_pitch: ", des_pitch, "current pitch: ", current_pitch)
    
    output_roll = K_P_2 * (des_roll - current_roll) + K_D_2 * (0 - angular_velocity[2])
    output_pitch = K_P_2 * (des_pitch - current_pitch) + K_D_2 * (0 - angular_velocity[1])  
    output_yaw = K_P_2 * (des_yaw - current_yaw) + K_D_2 * (0 - angular_velocity[3])
    output_thrust = (10*error_z - 10* v_z )+ thrust_feedforward
    # K_p_thrust * (altitude_ref - altitude) + K_d_thrust * (0 - get_state(environment)[9])+ thrust_feedforward
    println("output_roll: ", output_roll, " output_pitch: ", output_pitch, " output_yaw: ", output_yaw, " output_thrust: ", output_thrust)

    u = MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    print("u: ", u)
    set_input!(environment, u)  
end


function MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    u = zeros(4)
    
    u[1] = output_thrust - output_yaw - output_pitch - output_roll 
    u[2] = output_thrust - output_yaw + output_pitch - output_roll 
    u[3] = output_thrust + output_yaw - output_pitch + output_roll 
    u[4] = output_thrust - output_yaw + output_pitch + output_roll 


    # u[1] = 0
    # u[2] = 0
    # u[3] = 3
    # u[4] =0

    return u
end

initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)

### Visualize
vis = visualize(quadrotor_env)
render(vis)

# k: 120
# ref_position_xyz_world: [3.0, 0.0, 0.3]
# current position: [0.44535030375052803, -0.01163659928444805, 0.2964093541113269]
# states : (-0.007367388333140422, 0.013453785103226697, 0.00320730861181332, 0.2964093541113269)
# linear_velocity: 0.12209548448340081linear_velocity[1]: 0.8160458248439306
# roll ref: 0.011744084476962159, pitch_ref: 0.02058140536558581
# roll_ref: 0.011744084476962159 pitch_ref: roll: -0.007367388333140422 pitch: 0.013453785103226697 yaw_ref: 0 altitude_ref: 0.3
# roll_cntrl: -0.15905116819751836 pitch_cntrl: 0.15528532504559245 yaw_cntrl: -0.0005228500884081558 thrust: 50.24422625314279
# u: [49.92936690981127, 50.24098326007927, 50.558039896297494, 50.24851494638313]

# k 120
# Des_pos: [2.0, 0.0, 0.5]
# current_pos: [0.6483806285432365, 0.6915415945463479, 0.3300698537202133]
# des_roll: -0.15971995700974695current roll: -1.0282744954935343 des_pitch: -0.3064178817241348current pitch: 0.6241930268662458
# output_roll: 1.8705197974511876 output_pitch: -0.5005082662639757 output_yaw: -0.13151049440695672 output_thrust: 58.03948406918694
# u: [56.800983032406684, 55.79996649987873, 59.2779851059672, 60.54202262730906]Des_pos: [2.0, 0.0, 0.5]
# current_pos: [0.6703645504601635, 0.720470132213859, 0.32421903230753346]
# current_waypoint_index: 2
