using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

using Dojo
using DojoEnvironments
using Plots

# Load the robot arm
urdf = :youbot_arm_only_fixed_gripper
arm = get_mechanism(:youbot; urdf, gravity=[0; 0; -9.81], joint_limits=Dict(), timestep=0.001)

# Desired joint angles
qdes = [0.1, -0.5, 0.3, -0.2, 1]

# Initialize the arm with desired joint angles
initialize!(arm, :youbot; arm_angles=qdes)

# PD controller parameters
kp = [10.0, 10.0, 10.0, 10.0, 10.0]
kd = [1.0, 1.0, 1.0, 1.0, 1.0]

function pd_controller!(mechanism, k, qdes, kp, kd)
    state = get_minimal_state(mechanism)
    q = state[1:2:9] # joint angles
    dq = state[2:2:10] # joint angular velocities
    
    # PD control law
    error = qdes .- q
    derror = .-dq
    u = kp .* error .+ kd .* derror
    
    set_input!(mechanism, u)
end

# Function for PD control simulation
function run_simulation(arm, qdes, kp, kd)
    # arm.gravity = gravity
    initialize!(arm, :youbot; arm_angles=qdes)
    storage = Storage(5000, 5)
    storage = simulate!(arm, 1:5000, storage, (m, k) -> pd_controller!(m, k, qdes, kp, kd), record=true)
    return storage
end

# Simulate with normal gravity
storage_normal = run_simulation(arm, qdes, kp, kd)

# Simulate with zero gravity
storage_zero = run_simulation(arm, qdes, kp, kd, [0; 0; 0])

# Visualize results
vis_normal = visualize(arm, storage_normal)
render(vis_normal)
vis = visualize(arm, storage) # you can visualize the storage even if the simulation fails
render(vis)
vis_zero = visualize(arm, storage_zero)
render(vis_zero)