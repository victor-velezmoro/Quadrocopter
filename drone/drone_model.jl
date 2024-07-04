using Dojo
using DojoEnvironments
using LinearAlgebra


HORIZON=2000
quadrotor_env=get_environment(:quadrotor_waypoint;HORIZON)
ref_position_xyz_world = [0; 0; 1]
 next_waypoint=1

 state_traj=zeros(size(get_state(quadrotor_env))[1], HORIZON)
 reference_traj=zeros(4,Horizon)
 plot_dict = Dict("state_traj" => state_traj, "reference_traj" => reference_traj)

 function get_transformation_body_to_world(x, y, z, roll, pitch, yaw)
    #rotation matrix
    R=  RotXYZ(roll, pitch, yaw)
    R_matrix = convert(Array{Float64, 2}, R) #convert to a 3*3 R_matrix

    position_w = [x, y, z]

    # Translation matrix
    T_world_to_body = [
    R_matrix' -R_matrix' * position_w
    ]

    return T_world_to_body
 end
 function MMA!(roll, pitch, yaw)
    u=zeros(4)
    u[1]=thrust-pitch-yaw
    u[2]=thrust=roll+pitch-yaw
    u[3]=thrust-roll=pitch+yaw
    u[4]=thrust-roll-pitch-yaw
 end

 function sensing_and_estimation(environment)
    state=get_state(environment)
    state = get_state(environment)
    position = state[1:3] # x, y, z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    altitude = position[3]
    return roll, pitch, yaw, altitude
 end
 function position_to_quadrotor_orientation_controller(environment, k)
    