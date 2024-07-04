using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

using Dojo
using DojoEnvironments
using Plots

urdf = :youbot_arm_only_fixed_gripper
arm = get_mechanism(:youbot; urdf, gravity=0, joint_limits=Dict(), timestep=0.001) # Loads the youbot arm

function controller!(mechanism, k) # controller for the arm
    state = get_minimal_state(mechanism)
    q = state[1:2:9] # angles
    dq = state[2:2:10] # angular velocities
    
    u = zeros(5) # inputs for joint_1 to joint_5

    set_input!(mechanism, u)
end

arm.gravity = [0;0;-9.81] # set the gravity

qdes = zeros(5)

initialize!(arm, :youbot; arm_angles=qdes)

storage = Storage(5000,5)
storage = simulate!(arm, 1:5000, storage, controller!, record = true)
vis = visualize(arm, storage) # you can visualize the storage even if the simulation fails
render(vis)