using Dojo
using DojoEnvironments
using LinearAlgebra
using Rotations
using ReinforcementLearning

# Define the horizon
HORIZON = 150
quadrotor_env = get_environment(:quadrotor_waypoint; horizon=HORIZON)

# Define the state and action space
function env_config()
    return RLBase.ContinuousSpace((-Inf, Inf), 12), RLBase.ContinuousSpace((-1, 1), 4)
end

# Define the reward function
function reward_function(state, action, new_state, ref_position_xyz_world)
    position_error = norm(new_state[1:3] - ref_position_xyz_world)
    velocity_error = norm(new_state[7:9])
    orientation_error = norm(new_state[4:6])
    return -position_error - 0.1 * velocity_error - 0.1 * orientation_error
end

# Define the RL environment
struct QuadrotorEnv <: AbstractEnv
    environment::Environment
    ref_position_xyz_world::Vector{Float64}
end

function RLBase.reset!(env::QuadrotorEnv)
    initialize!(env.environment, :quadrotor)
    env.ref_position_xyz_world = [0; 0; 0]
    return get_state(env.environment)
end

function RLBase.step!(env::QuadrotorEnv, action)
    u = MMA!(action[1], action[2], action[3], action[4])
    set_input!(env.environment, u)
    Dojo.step!(env.environment)
    new_state = get_state(env.environment)
    reward = reward_function(get_state(env.environment), action, new_state, env.ref_position_xyz_world)
    is_done = RLBase.termination_status(env)
    return new_state, reward, is_done, []
end

function RLBase.termination_status(env::QuadrotorEnv)
    return RLBase.TerminationState(Done())
end

# Define the MMA function
function MMA!(roll, pitch, yaw, thrust)
    u = zeros(4)
    u[1] = thrust + roll - pitch + yaw
    u[2] = thrust + roll + pitch - yaw
    u[3] = thrust - roll + pitch + yaw
    u[4] = thrust - roll - pitch - yaw
    return u
end

# Initialize the RL environment
rl_env = QuadrotorEnv(quadrotor_env, [0; 0; 0])

# Define the RL agent
policy = RandomPolicy(rl_env)
agent = Agent(rl_env, policy)

# Train the RL agent
for episode in 1:1000
    total_reward = 0
    state = reset!(rl_env)
    for t in 1:HORIZON
        action = policy(state)
        new_state, reward, is_done, _ = step!(rl_env, action)
        total_reward += reward
        state = new_state
        if is_done
            break
        end
    end
    println("Episode $episode: Total Reward: $total_reward")
end

# Use the trained RL agent to control the quadrotor
function rl_controller!(environment, k)
    global ref_position_xyz_world
    state = get_state(environment)
    action = policy(state)
    u = MMA!(action[1], action[2], action[3], action[4])
    set_input!(environment, u)
end

# Initialize and simulate the environment with the RL controller
initialize!(quadrotor_env, :quadrotor)
simulate!(quadrotor_env, rl_controller!; record=true)

vis = visualize(quadrotor_env)
render(vis)
