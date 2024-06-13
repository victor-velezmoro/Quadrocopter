using Pkg
Pkg.activate(@__DIR__)
Pkg. instantiate()

using Dojo
using DojoEnvironments
using Plots

urdf = :youbot_arm_only_fixed_gripper
arm = get_mechanism(:youbot; urdf, gravity = [0; 0; -9.81], joint_limits=Dict(),timestep=0.01)

kp = [10.0, 10.0, 10.0, 10.0, 10.0]
kd = [1.0, 1.0, 1.0, 1.0, 1.0]
timestep = 0.01

function qdes_trajectory(t)
    return [0.1 * sin(t), -0.5 * cos(t), 0.3 * sin(t), -0.2 * cos(t), 0.1 * sin(t)]
end

initialize!(arm, :youbot; arm_angles=qdes_trajectory(0.0))

function pd_trajectory_controller!(mechanism, k, qdes_trajectory, kp, kd, timestep)
    state = get_minimal_state(mechanism)
    q = state[1:2:9]
    dq = state[2:2:10]

    t = kd * timestep
    qdes = qdes_trajectory(t)

    error = qdes .- q
    derror = .-dq
    u = kp .* error .+ kd .* derror

    set_input!(mechanism, u)
end



function run_simulation(arm, qdes_trajectory, kp, kd, timestep)
    initialize!(arm, :youbot; arm_angles=qdes_trajectory(0.0))
    storage = Storage(5000, 5)
    storage = simulate!(arm, 1:5000, storage, (m, k) -> pd_trajectory_controller!(m, k, qdes_trajectory, kp, kd, timestep), record=true)
    return storage
end


storage = run_simulation(arm, qdes_trajectory, kp, kd, timestep)

vis_normal = visualize(arm, storage_normal)
render(vis_normal)