using Dojo
using Flux

origin = Origin()
body = Sphere(0.25, 1.0)
joint = JointConstraint(Prismatic(origin, body, [0,1,0]; parent_vertex=[0,0,0.5]))
mechanism = Mechanism(origin, [body], [joint], timestep=0.2)

function controller!(mechanism, k)
    state = get_minimal_state(mechanism)  # position, velocity

    u = zeros(1)

    set_input!(mechanism, u)
end

set_minimal_coordinates!(mechanism, mechanism.joints[1], [0])
set_minimal_velocities!(mechanism, mechanism.joints[1], [0])

storage = simulate!(mechanism, 5.0, controller!, record=true)
vis = visualize(mechanism, storage)
render(vis)

model = Chain(Dense(3 => 1, relu), Dense(1 => 1, tanh), only)
data = [(rand(3), rand())]
optim = Flux.setup(Adam(), model)

Flux.train!((m,x,y) -> (m(x) - y)^2, model, data, optim)