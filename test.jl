using Dojo
using DojoEnvironments

quadrotor_env = get_environment(:quadrotor_waypoint; horizon=1000)

function controller!(environment, k)
    state = get_state(environment)
    position = state[1:3] # x,y,z
    orientation = state[4:6] # axis*angle
    linear_velocity = state[7:9] # vx, vy, vz
    angular_velocity = state[10:12] # ωx, ωy, ωz 

    rotor_speeds = [20;30;40;50] # just for demonstration, does not make sense

    set_input!(environment, rotor_speeds) # rotor speeds are directly set
end

initialize!(quadrotor_env, :quadrotor; body_orientation=Dojo.RotZ(pi/2))
simulate!(quadrotor_env, controller!; record=true)
vis = visualize(quadrotor_env) # you can visualize even if the simulation fails
render(vis)
