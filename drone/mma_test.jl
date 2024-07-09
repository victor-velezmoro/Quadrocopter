using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations

HORIZON = 2000
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=HORIZON)
ref_position_xyz_world = [0;0;0]
next_waypoint = 1
state_traj = zeros(size(get_state(quadrotor_env))[1], HORIZON)

reference_traj = zeros(4, HORIZON)
plot_dict = Dict("state_traj" => state_traj, "reference_traj" => reference_traj)

# function get_transformation_body_to_world(x, y, z, roll, pitch, yaw)
#     R = RotXYZ(roll, pitch, yaw)
#     R_matrix = convert(Array{Float64, 2}, R) # Convert to matrix
#     println("R_matrix: ", R_matrix)
    
#     position_w = [x, y, z]
#     T_world_to_body = [R_matrix' -R_matrix' * position_w; 0 0 0 1]
#     return T_world_to_body
# end

function MMA!(roll, pitch, yaw, thrust)
    u = zeros(4)
    u[1] = thrust + roll - pitch + yaw
    u[2] = thrust + roll + pitch - yaw
    u[3] = thrust - roll + pitch + yaw
    u[4] = thrust - roll - pitch - yaw
    return u
end

function sensing_and_estimation(environment)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 
    
    altitude = position[3]
    roll, pitch, yaw = orientation
    
    return roll, pitch, yaw, altitude
end

function position_to_quadrotor_orientation_controller(environment, k)
    global ref_position_xyz_world
    position = get_state(environment)[1:3]
    roll, pitch, yaw = get_state(environment)[4:6]
    linear_velocity = get_state(environment)[7:9]
    
    # transformation_b_w = get_transformation_body_to_world(position[1], position[2], position[3], roll, pitch, yaw) 
    # println("transformation_b_w: ", transformation_b_w)
    # ref_position_b = transformation_b_w * [ref_position_xyz_world;1]
    # ref_position_b = ref_position_b[1:3]
    # ref_position_b = ref_position_xyz_world
    # println("current position: , $position")
    # plot_dict["state_traj"][:, k] = get_state(environment)
    # println("ref_position_b: ", ref_position_b)  
   
    K_p_xy_roll = K_p_xy_pitch = 0.04
    K_d_xy_roll = K_d_xy_pitch = 0.1
   
    roll_ref = -(K_p_xy_roll * (ref_position_xyz_world[2] - position[2]) + K_d_xy_roll * (0 - linear_velocity[2]))
    pitch_ref = K_p_xy_pitch * (ref_position_xyz_world[1] - position[1]) + K_d_xy_pitch * (0 - linear_velocity[1])
    #plot_dict["reference_traj"][1:3, k] = [roll_ref, pitch_ref, ref_position_xyz_world[2]-position[2]]
   
    
    return roll_ref, pitch_ref
end 



function cascade_controller!(environment, k)
    global ref_position_xyz_world
    # get the current system values
    roll, pitch, yaw, altitude = sensing_and_estimation(environment)
    println("states : $(sensing_and_estimation(environment))")
    # set some reference - TODO - make this to be set from outside or smth
    #roll_ref, pitch_ref, yaw_ref, altitude_ref = [0.0, 0.0, 1.57, 1]
    roll_ref, pitch_ref, = position_to_quadrotor_orientation_controller(environment, k)
    println("roll ref: $roll_ref, pitch_ref: $pitch_ref")
    yaw_ref = 0 #1.57
    altitude_ref = ref_position_xyz_world[3] #1

    # PIDs
    # roll
    K_p_roll = 4 #1
    K_d_roll = 1
    # pitch
    K_p_pitch = 4 #1
    K_d_pitch = 1
    # yaw
    K_p_yaw = 4 #1
    K_d_yaw = 1
    # thrust
    K_p_thrust = 10 # kinda tuned
    K_d_thrust = 10 # kinda tuned
    # thrust feedforward is there to compensate gravity and i took the numbers from the cascaded hover example.
    thrust_feedforward = 20 * 5.1 * 1/sqrt(4)#normalize([1;1;1;1])

    roll_cntrl = K_p_roll * (roll_ref - roll) + K_d_roll * (0 - get_state(environment)[10])
    pitch_cntrl = K_p_pitch * (pitch_ref - pitch) + K_d_pitch * (0 - get_state(environment)[11])
    yaw_cntrl = K_p_yaw * (yaw_ref - yaw) + K_d_yaw * (0 - get_state(environment)[12])
    thrust = K_p_thrust * (altitude_ref - altitude) + K_d_thrust * (0 - get_state(environment)[9])+ thrust_feedforward
    println("roll_cntrl: ", roll_cntrl, " pitch_cntrl: ", pitch_cntrl, " yaw_cntrl: ", yaw_cntrl, " thrust: ", thrust)
    u = MMA!(roll_cntrl, pitch_cntrl, yaw_cntrl, thrust)
    println("u: ", u)
    set_input!(environment, u)

end

function controller!(environment, k)
    println("k: ", k)
    fly_through_waypoints_controller!(environment, k) #needed here to define the goal position
end

function default_controller!(environment, k)
    set_input!(environment, rotor_speeds)
end

function fly_through_waypoints_controller!(environment, k)
    global next_waypoint
    global ref_position_xyz_world
    waypoints = 
    [
        [1;1;0.3],
        [2;0;0.3],
        [1;-1;0.3],
        [0;0;0.3],
    ]
    if norm(get_state(environment)[1:3]-waypoints[next_waypoint]) < 1e-1
        if next_waypoint < 4
            println("next_waypoint: ", next_waypoint)
            next_waypoint += 1
        end
    end
    ref_position_xyz_world = waypoints[next_waypoint]
    println("ref_position_xyz_world: ", ref_position_xyz_world)
    println("current position: ", get_state(environment)[1:3])
    cascade_controller!(environment, k)
end

initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, controller!; record=true)

vis = visualize(quadrotor_env)
render(vis)
