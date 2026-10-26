#pragma once

#include <Eigen/Core>
#include <array>

namespace locomotion {
using Vec3 = Eigen::Vector3d;
using Wrench = Eigen::Matrix<double, 6, 1>;
struct ForceSolution {
  std::array<Vec3, 4> forces;
  double residual = 0;
};

// Projected gradient least squares for four point contacts on horizontal ground.
// A conservative square friction pyramid is INSIDE the Coulomb cone.
class ForceAllocator {
 public:
  ForceSolution solve(const std::array<Vec3, 4>& arms,
                       const std::array<bool, 4>& stance, const Wrench& desired) const;
};
}  // namespace locomotion
