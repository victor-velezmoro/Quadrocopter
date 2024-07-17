using Dojo
using DojoEnvironments
using LinearAlgebra

quadrotor_env = get_environment(:quadrotor_waypoint; horizon=3000)
current_waypoint_index = 0
des_pos = [0; 0; 0]
final_landing = false

function controller!(environment, k)
    global current_waypoint_index
    check_waypoints!(environment, k)
end

function convert_axis_angle_to_euler(state)
    orientation = state[4:6]
    angle = norm(orientation)
    axis = orientation / angle
    R = AngleAxis(angle, axis[1], axis[2], axis[3])
    rotation_matrix = RotMatrix(R)
    euler_angles = euler(rotation_matrix, ZYX)
    return euler_angles
end

function check_waypoints!(environment, k)
    global current_waypoint_index
    global des_pos
    global final_landing

    waypoints = [[1.0, 1.0, 0.5], [2.0, 0.0, 0.5], [1.0, -1.0, 0.5], [0.0, 0.0, 0.5]]
    current_pos = get_state(environment)[1:3]

    if norm(current_pos - des_pos) < 0.1 && !final_landing
        if current_waypoint_index < length(waypoints)
            current_waypoint_index += 1
            println("Next waypoint: ", waypoints[current_waypoint_index])
        else
            final_landing = true
            println("Landing at final position")
        end
    end

    if final_landing
        des_pos = [current_pos[1], current_pos[2], 0.0]
    else
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
    position = state[1:3]
    orientation = state[4:6]
    linear_velocity = state[7:9]
    angular_velocity = state[10:12]

    current_roll = orientation[1]
    current_pitch = orientation[2]
    current_yaw = orientation[3]

    altitude = position[3]

    return position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index
end

function position_controller!(environment, k)
    global des_pos
    global current_waypoint_index

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    K_P_1 = 0.04
    K_D_1 = 0.1
    des_vel = des_pos .- position

    des_pitch = (0.01 * (des_vel[1] - linear_velocity[1]) + K_D_1 * (0 - linear_velocity[1]) + (0.04 * (des_pos[1] - position[1])))
    des_roll = -((0.01 * (des_vel[2] - linear_velocity[2]) + K_D_1 * (0 - linear_velocity[2]) + (0.04 * (des_pos[2] - position[2]))))

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

    position, orientation, linear_velocity, angular_velocity, current_roll, current_pitch, current_yaw, altitude, des_pos, current_waypoint_index = state_provider!(environment)

    error_z = des_pos[3] - altitude
    v_z = linear_velocity[3]
    des_yaw = 0.0
    K_p_thrust = 10 
    K_d_thrust = 10
    des_all = des_pos[3] - position[3]

    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)
    gravity = 9.81
    mass = 1.04
    num_rotors = 4

    total_weight = mass * gravity
    force = total_weight / num_rotors
    force_factor = 0.001
    rpm = sqrt(abs(force) / force_factor)
    rpm_new = sign(force) * rpm
    push!(rpm_new_list, rpm_new)

    des_roll, des_pitch = position_controller!(environment, k)
    output_roll = 3 * (des_roll - current_roll) + 1 * (0 - angular_velocity[1])
    output_pitch = 3 * (des_pitch - current_pitch) + 1 * (0 - angular_velocity[2])
    output_yaw = 3 * (des_yaw - current_yaw) + 1 * (0 - angular_velocity[3])
    output_thrust = 5 * ((des_all + 0.3) - altitude) + 5 * (0 - get_state(environment)[9]) + 3 * (error_z - linear_velocity[3]) + rpm_new

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

    u = MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    set_input!(environment, u)
end

function MMA!(output_roll, output_pitch, output_yaw, output_thrust)
    u = zeros(4)

    u[3] = output_thrust + output_roll + output_pitch + 0
    u[4] = output_thrust - output_roll + output_pitch - 0
    u[2] = output_thrust + output_roll - output_pitch - 0
    u[1] = output_thrust - output_roll - output_pitch + 0

    return u
end

initial_position = 1
initial_orientation = Dojo.RotZ(-π/4)

initialize!(quadrotor_env, :quadrotor, body_orientation=Dojo.RotZ(-π/4))
simulate!(quadrotor_env, controller!; record=true)

vis = visualize(quadrotor_env)
render(vis)
