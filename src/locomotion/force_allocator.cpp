#include "locomotion/force_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace locomotion {
namespace {
Vec3 project_friction_pyramid(const Vec3& force) {
  constexpr double mu = 0.5, max_normal = 160;
  // Exact Euclidean projection: enumerate the four active tangential sets.
  Vec3 best = Vec3::Zero();
  double best_cost = force.squaredNorm();
  for (int mask = 0; mask < 4; ++mask) {
    const int nx = mask & 1, ny = (mask >> 1) & 1;
    const double z = std::clamp((force.z() + mu * (nx * std::abs(force.x()) + ny * std::abs(force.y()))) /
                                (1 + mu * mu * (nx + ny)), 0.0, max_normal);
    const Vec3 candidate(std::clamp(force.x(), -mu * z, mu * z),
                         std::clamp(force.y(), -mu * z, mu * z), z);
    const double cost = (candidate - force).squaredNorm();
    if (cost < best_cost) { best = candidate; best_cost = cost; }
  }
  return best;
}
}  // namespace
ForceSolution ForceAllocator::solve(const std::array<Vec3, 4>& arms,
                                    const std::array<bool, 4>& stance,
                                    const Wrench& desired) const {
  if (!desired.allFinite()) throw std::invalid_argument("Non-finite desired wrench");
  Eigen::Matrix<double, 6, 12> a = Eigen::Matrix<double, 6, 12>::Zero();
  for (int i = 0; i < 4; ++i) {
    if (!arms[i].allFinite()) throw std::invalid_argument("Non-finite contact arm");
    if (!stance[i]) continue;
    const auto& r = arms[i];
    a.block<3, 3>(0, 3 * i).setIdentity();
    a.block<3, 3>(3, 3 * i) << 0, -r.z(), r.y(), r.z(), 0, -r.x(), -r.y(), r.x(), 0;
  }
  // Moments are in N.m; scaling increases their priority relative to net force.
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1, 1, 1, 5, 5, 3;
  const auto aw = (weights.asDiagonal() * a).eval();
  const auto bw = (weights.asDiagonal() * desired).eval();
  Eigen::Matrix<double, 12, 12> h = aw.transpose() * aw;
  h.diagonal().array() += 0.02;
  const Eigen::Matrix<double, 12, 1> b = aw.transpose() * bw;
  // Infinity norm upper-bounds the Hessian spectral radius.
  const double step = 1.0 / h.cwiseAbs().rowwise().sum().maxCoeff();
  Eigen::Matrix<double, 12, 1> f = Eigen::Matrix<double, 12, 1>::Zero();
  for (int k = 0; k < 100; ++k) {
    f -= step * (h * f - b);
    for (int i = 0; i < 4; ++i) {
      auto fi = f.segment<3>(3 * i);
      if (!stance[i]) { fi.setZero(); continue; }
      fi = project_friction_pyramid(fi);
    }
  }
  ForceSolution result;
  for (int i = 0; i < 4; ++i) result.forces[i] = f.segment<3>(3 * i);
  result.residual = (aw * f - bw).norm();
  return result;
}
}  // namespace locomotion
