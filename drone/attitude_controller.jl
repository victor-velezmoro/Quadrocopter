using Dojo
using DojoEnvironments
using LinearAlgebra
using PyPlot


#TO DO 
#Special featurs a flip or something else
#make drone terminate in final position
#creation of nice video 
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=600)
current_waypoint_index = 0
des_pos = [0;0;0]

function controller!(environment, k)
    global current_waypoint_index
    check_waypoints!(environment, k)
end

#not used 
function convert_axis_angle_to_euler(state)
    # Extract the axis-angle representation from the state vector
    orientation = state[4:6]
    angle = norm(orientation)  # The angle is the norm of the axis*angle representation
    axis = orientation / angle  # Normalize to get the axis

    # Create an AngleAxis object
    R = AngleAxis(angle, axis[1], axis[2], axis[3])
    roll, pitch, yaw = rotation_angel(R)
    println("roll: ", roll, " pitch: ", pitch, " yaw: ", yaw)
    # Convert AngleAxis to a rotation matrix
    rotation_matrix = RotMatrix(angle_axis)

    euler_angles = euler(rotation_matrix, ZYX)  # Default order is ZYX for roll, pitch, yaw

    return euler_angles
end


function check_waypoints!(environment, k)

    global current_waypoint_index
    global des_pos
     waypoints = 
    [[0, 0.0, 3], [2.0, 0.0, 0.5], [1.0, -1.0, 0.5], [0.0, 0.0, 0.5]]
    
 
    current_pos = get_state(environment)[1:3]
    if norm(current_pos - des_pos) < 0.1 && current_waypoint_index < length(waypoints)
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

    return position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index
end


function position_controller!(environment, k)
    # here we will get the current_roll, current_pitch, current_yaw and thrust commands

    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    
    #This loop translates position errors into desired roll angles.
    K_P_1 = 0.01
    K_D_1= 0.1
    K_I_1 = 0.04
    des_vel = des_pos .- position
    println("k ", k)
    println("Des_pos: ", des_pos)
    println("current_pos: ", position)


    des_pitch= (K_P_1 * (des_vel[1] - linear_velocity[1]) + K_D_1 * (0 - linear_velocity[1]) + (K_I_1 * (des_pos[1] - position[1])))
    des_roll = -((K_P_1 * (des_vel[2] - linear_velocity[2]) + K_D_1 * (0 - linear_velocity[2]) + (K_I_1 * (des_pos[2] - position[2]))))
    # des_pitch= (K_P_1 * (des_vel[1] - linear_velocity[1]) + K_D_1 * (0 - linear_velocity[1]))
    # des_roll = -((K_P_1 * (des_vel[2] - linear_velocity[2]) + K_D_1 * (0 - linear_velocity[2])))
    # des_pitch= ( + K_D_1 * (0 - linear_velocity[1]) + (K_I_1 * (des_pos[1] - position[1])))
    # des_roll = -( + K_D_1 * (0 - linear_velocity[2]) + (K_I_1 * (des_pos[2] - position[2])))

    # print("des_roll: ", des_roll, " des_pitch: ", des_pitch)

    return des_roll, des_pitch

end

global rpm_new_list = []
global des_roll_list = []
global current_roll_list = []
global angular_velocity_roll_list = []
global output_roll_list = []
global des_pitch_list = []
global current_pitch_list = []
global angular_velocity_pitch_list = []
global output_pitch_list = []
global output_yaw_list = []
global output_thrust_list = []
global des_altitude_list = []
global position_list = []
global all_error_list = []
global z_ang_vel_list = []
global des_yaw_list = []
global current_yaw_list = []
global angular_velocity_yaw_list = []
global output_yaw_list = []

function attitude_controller!(environment, k)
    global des_pos
    global current_waypoint_index
    global rpm_new_list
    global des_roll_list
    global current_roll_list
    global angular_velocity_roll_list
    global output_roll_list
    global des_pitch_list
    global current_pitch_list
    global angular_velocity_pitch_list
    global output_pitch_list
    global output_yaw_list
    global output_thrust_list
    global des_altitude_list
    global position_list
    global all_error_list
    global z_ang_vel_list
    global des_yaw_list
    global current_yaw_list
    global angular_velocity_yaw_list
    global output_yaw_list


    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    error_z = des_pos[3] - altitude
    v_z = linear_velocity[3]
    des_yaw = 0.0
    des_all = des_pos[3] -position[3] 

    #Calculationg the thurst by using mass and gravity 
    gravity = 9.81  # Acceleration due to gravity in m/s^2
    mass = 1.04     # Mass of the drone in kg
    num_rotors = 4  # Number of rotors
    total_weight = mass * gravity
    force = total_weight / num_rotors
    force_factor = 0.001
    rpm = sqrt(abs(force) / force_factor)
    rpm_new = sign(force) * rpm 
    push!(rpm_new_list, rpm_new)
    

    des_roll, des_pitch = position_controller!(environment, k)
    #Thrust output 
    output_thrust = 5 * ((des_all+0.3) - altitude) + 5 * (0 - get_state(environment)[9]) + 3 *(error_z - linear_velocity[3])+ rpm_new 
    #Output roll, pitch and yaw
    output_roll = 3 * (des_roll - current_roll) + 1 * (0 - angular_velocity[1])
    output_pitch = 3 * (des_pitch - current_pitch) + 1 * (0 - angular_velocity[2])  
    output_yaw = 3 * (des_yaw - current_yaw) + 1 * (0 - angular_velocity[3])

    # println("thrust: ", output_thrust)



    # Store values in lists
    push!(des_roll_list, des_roll)
    push!(current_roll_list, current_roll)
    push!(angular_velocity_roll_list, angular_velocity[1])
    push!(output_roll_list, output_roll)
    push!(des_pitch_list, des_pitch)
    push!(current_pitch_list, current_pitch)
    push!(angular_velocity_pitch_list, angular_velocity[2])
    push!(des_altitude_list, des_pos[3])
    push!(output_pitch_list, output_pitch)
    push!(output_yaw_list, output_yaw)
    push!(output_thrust_list, output_thrust)
    push!(position_list, altitude)
    push!(all_error_list, des_all)
    push!(z_ang_vel_list, linear_velocity[3])
    push!(des_yaw_list, des_yaw)
    push!(current_yaw_list, current_yaw)
    push!(angular_velocity_yaw_list, angular_velocity[3])
    push!(output_yaw_list, output_yaw)

    rotor1, rotor2, rotor3, rotor4 = MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    rotor_speed = [rotor1, rotor2, rotor3, rotor4]
    set_input!(environment, rotor_speed)  
end

function plot_attitude_controller_data()
    global rpm_new_list
    global des_roll_list
    global current_roll_list
    global angular_velocity_roll_list
    global output_roll_list
    global des_pitch_list
    global current_pitch_list
    global angular_velocity_pitch_list
    global output_pitch_list
    global output_yaw_list
    global output_thrust_list
    global des_altitude_list
    global all_error_list
    global z_ang_vel_list
    global des_yaw_list
    global current_yaw_list
    global angular_velocity_yaw_list
    global output_yaw_list

    figure()
    subplot(2, 2, 1)
    plot(des_roll_list, label="Desired Roll")
    plot(current_roll_list, label="Current Roll")
    plot(angular_velocity_roll_list, label="Angular Velocity Roll")
    plot(output_roll_list, label="Output Roll")
    # legend()
    title("Roll Control")

    subplot(2, 2, 2)
    plot(des_pitch_list, label="Desired Pitch")
    plot(current_pitch_list, label="Current Pitch")
    plot(angular_velocity_pitch_list, label="Angular Velocity Pitch")
    plot(output_pitch_list, label="Output Pitch")
    # legend()
    title("Pitch Control")

    subplot(2, 2, 3)
    plot(des_yaw_list)
    plot(current_yaw_list)
    plot(angular_velocity_yaw_list)
    plot(output_yaw_list)
    title("Yaw Control")

    # subplot(2, 2, 4)
    # # plot(output_thrust_list, label="Output Thrust")
    # # plot(rpm_new_list, label="RPM New")
    # plot(des_pitch_list, label="Desired Pitch")
    # plot(current_pitch_list, label="Current Pitch")
    # plot(angular_velocity_pitch_list, label="Angular Velocity Pitch")
    # plot(output_pitch_list, label="Output Pitch")
    # title("Thrust Control")
    subplot(2, 2, 4)
    # plot(output_thrust_list, label="Output Thrust")
    # plot(rpm_new_list, label="RPM New")
    plot(des_altitude_list, label="Desired Altitude")
    plot(position_list, label="Current Altitude")
    plot(all_error_list, label="Error")
    plot(z_ang_vel_list, label="Z Angular Velocity")
    #plot(output_thrust_list, label="Output Thrust")
    title("Thrust Control")


    tight_layout()
    savefig("altitude_test.png")
end


function MMA!(output_roll, output_pitch, output_yaw, output_thrust)

    rotor3 = output_thrust + output_roll + output_pitch + output_yaw
    rotor4 = output_thrust - output_roll + output_pitch - output_yaw
    rotor2 = output_thrust + output_roll - output_pitch - output_yaw
    rotor1 = output_thrust - output_roll - output_pitch + output_yaw

    return rotor1, rotor2, rotor3, rotor4
end

# Define the initial position and orientation
initial_position = 1  # Replace with your desired initial position
initial_orientation = Dojo.RotZ(-π/4)

# Initialize the environment with the specified initial position and orientation
initialize!(quadrotor_env, :quadrotor, body_orientation= Dojo.RotZ(0))
# initialize!(quadrotor_env, :quadrotor, body_orientation=Dojo.RotZ(-π/4))
simulate!(quadrotor_env, controller!; record=true)

plot_attitude_controller_data()
println("Done")

## Visualize
vis = visualize(quadrotor_env)
render(vis)

