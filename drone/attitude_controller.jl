using Dojo
using DojoEnvironments
using LinearAlgebra
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=500)
current_waypoint_index = 0
des_pos = [0;0;0]

function controller!(environment, k)
    global current_waypoint_index
    check_waypoints!(environment, k)
end

function convert_axis_angle_to_euler(state)
    # Extract the axis-angle representation from the state vector
    orientation = state[4:6]
    angle = norm(orientation)  # The angle is the norm of the axis*angle representation
    axis = orientation / angle  # Normalize to get the axis

    # Create an AngleAxis object
    R = AngleAxis(angle, axis[1], axis[2], axis[3])

    roll, pitch, yaw = rotation_angel(R)

    println("roll: ", roll, " pitch: ", pitch, " yaw: ", yaw)



    # # Convert AngleAxis to a rotation matrix
    # rotation_matrix = RotMatrix(angle_axis)

    # # Convert the rotation matrix to Euler angles (roll, pitch, yaw)
    # euler_angles = euler(rotation_matrix, ZYX)  # Default order is ZYX for roll, pitch, yaw

    return euler_angles
end

function check_waypoints!(environment, k)

    global current_waypoint_index
    global des_pos
     waypoints = 
    #  [[1.0, 1.0, 0.5], [2.0, -1.0, 0.5], [1.0, -1.0, 0.5], [0.0, 1.0, 0.5]]
    [
        [1;0;0.5],
    ]

 
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


    des_pitch = (K_P_1 * (des_pos[1]-position[1]) + K_D_1 * (0 - linear_velocity[1]))
    des_roll = -(K_P_1 * (des_pos[2]-position[2]) + K_D_1 * (0 - linear_velocity[2]))

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
    K_p_thrust = 10 
    K_d_thrust = 10
    des_all = des_pos[3] -position[3] 

    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)
    gravity = 9.81  # Acceleration due to gravity in m/s^2
    mass = 1.04     # Mass of the drone in kg
    num_rotors = 4  # Number of rotors

    # Calculate total gravitational force acting on the drone
    total_weight = mass * gravity

    # Calculate thrust required per rotor for hover
    force = total_weight / num_rotors
    force_factor = 0.001
    rpm = sqrt(abs(force) / force_factor)
    rpm_new = sign(force) * rpm
    println("rpm_new: ", rpm_new)


    des_roll, des_pitch = position_controller!(environment, k)
    println("des_roll: ", des_roll, "current roll: ", current_roll, " des_pitch: ", des_pitch, "current pitch: ", current_pitch)
    
    output_roll = K_P_2 * (des_roll - current_roll) + K_D_2 * (0 - angular_velocity[1])
    output_pitch = K_P_2 * (des_pitch - current_pitch) + K_D_2 * (0 - angular_velocity[2])  
    output_yaw = K_P_2 * (des_yaw - current_yaw) + K_D_2 * (0 - angular_velocity[3])
    #output_thrust = (10*error_z - 10* v_z )+ thrust_feedforward
    output_thrust = 1 * (des_all - altitude) + 2 * (0 - get_state(environment)[9]) + rpm_new
    # output_thrust = rpm_new
    println("output_roll: ", output_roll, " output_pitch: ", output_pitch, " output_yaw: ", output_yaw, " output_thrust: ", output_thrust)

    u = MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    print("u: ", u)
    set_input!(environment, u)  
end


function MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    u = zeros(4)
    
    # u[1] = output_thrust - output_yaw - output_pitch - output_roll 
    # u[2] = output_thrust - output_yaw + output_pitch - output_roll 
    # u[3] = output_thrust + output_yaw - output_pitch + output_roll 
    # u[4] = output_thrust - output_yaw + output_pitch + output_roll 

    u[3] = output_thrust + output_roll + output_pitch + 0
    u[4] = output_thrust - output_roll + output_pitch - 0
    u[2] = output_thrust + output_roll - output_pitch - 0
    u[1] = output_thrust - output_roll - output_pitch + 0

    # u[1] = 50
    # u[2] = 50
    # u[3] = 55
    # u[4] = 55


    return u
end

initialize!(quadrotor_env, :quadrotor, body_orientation=Dojo.RotZ(-π/4))
simulate!(quadrotor_env, controller!; record=true)

### Visualize
vis = visualize(quadrotor_env)
render(vis)

