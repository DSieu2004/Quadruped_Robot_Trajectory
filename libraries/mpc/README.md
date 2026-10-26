# MPC controller library

`quadruped::mpc` is a ROS- and simulator-independent C++17 library for
receding-horizon control of scalar double-integrator channels. Use one instance
per independently controlled axis.

For each future step, the controller predicts

```text
p[k+1] = p[k] + dt v[k] + 0.5 dt² a[k]
v[k+1] = v[k] + dt a[k]
```

and minimizes the finite-horizon quadratic cost

```text
J = Σ (Qp (p[k] - p_ref[k])²
      + Qv (v[k] - v_ref[k])²
      + R a[k]²
      + S (a[k] - a[k-1])²)
```

The acceleration sequence is constrained to `[acceleration_min,
acceleration_max]`. A projected-gradient solver computes the sequence; only its
first acceleration is applied. The solution is shifted as a warm start for the
next update. `S` penalizes acceleration changes, and the slew limit bounds the
change per adjacent sample so the optimizer cannot switch abruptly between
opposite acceleration limits.

```cpp
#include "mpc/mpc_controller.hpp"

mpc::MpcConfig config;
config.horizon_steps = 15;
config.position_weight = 20.0;
config.velocity_weight = 3.0;
config.acceleration_weight = 0.5;
config.acceleration_change_weight = 2.0;
config.acceleration_change_limit = 0.04;
config.acceleration_min = -0.15;
config.acceleration_max = 0.15;

mpc::MpcController controller(config);
std::vector<mpc::Reference> references(config.horizon_steps);
// Fill the vector with future position and velocity samples.
const double acceleration = controller.update(position, velocity, references, dt);
```

The reference vector starts one control period in the future and must contain
exactly `horizon_steps` samples. The trajectory follower uses independent MPC
channels for world-frame x, y and heading. The Go2 bridge also uses MPC channels
for body translation, attitude and swing-foot tracking, then maps the requested
wrench/foot forces through its contact-force allocator to joint torques. Gait
phase/contact scheduling remains a separate locomotion mechanism.
