#pragma once
// Minimal rigid-transform (SE(3)) helpers matching nalgebra::Isometry3<f32>'s
// composition convention exactly (hop-bench-cxx's CPose is just that struct's
// fields laid out flat) so pose algebra done here composes identically with
// poses that came from or are going into the Rust side.
#include "hop_bench_cxx.h"
#include <cmath>
#include <random>

inline CPose pose_identity() { return CPose{0, 0, 0, 0, 0, 0, 1}; }

// Matches hop_bench.Pose.from_xyz_yaw (translation + rotation about world Z).
inline CPose pose_from_xyz_yaw(float x, float y, float z, float yaw) {
  return CPose{x, y, z, 0, 0, std::sin(yaw / 2.0f), std::cos(yaw / 2.0f)};
}

inline CPose pose_mul(CPose a, CPose b);

// Matches hop_bench.Pose.from_xyz_rpy / nalgebra::UnitQuaternion::from_euler_angles's
// convention (R = Rz(yaw) * Ry(pitch) * Rx(roll)).
inline CPose pose_from_xyz_rpy(float x, float y, float z, float roll, float pitch, float yaw) {
  CPose qx{0, 0, 0, std::sin(roll / 2.0f), 0, 0, std::cos(roll / 2.0f)};
  CPose qy{0, 0, 0, 0, std::sin(pitch / 2.0f), 0, std::cos(pitch / 2.0f)};
  CPose qz{0, 0, 0, 0, 0, std::sin(yaw / 2.0f), std::cos(yaw / 2.0f)};
  CPose rot = pose_mul(pose_mul(qz, qy), qx);
  return CPose{x, y, z, rot.qx, rot.qy, rot.qz, rot.qw};
}

// Hamilton quaternion product convention, matching nalgebra. a * b: apply b
// first, then a (i.e. this is `a.compose(b)` / standard SE(3) `a * b`).
inline CPose pose_mul(CPose a, CPose b) {
  float ax = a.qx, ay = a.qy, az = a.qz, aw = a.qw;
  float bx = b.qx, by = b.qy, bz = b.qz, bw = b.qw;
  CPose r{};
  r.qw = aw * bw - ax * bx - ay * by - az * bz;
  r.qx = aw * bx + ax * bw + ay * bz - az * by;
  r.qy = aw * by - ax * bz + ay * bw + az * bx;
  r.qz = aw * bz + ax * by - ay * bx + az * bw;
  // rotate b's translation by a's rotation, then add a's translation
  float tx = b.x, ty = b.y, tz = b.z;
  float uvx = ay * tz - az * ty, uvy = az * tx - ax * tz, uvz = ax * ty - ay * tx;
  float uuvx = ay * uvz - az * uvy, uuvy = az * uvx - ax * uvz, uuvz = ax * uvy - ay * uvx;
  r.x = a.x + tx + 2.0f * (aw * uvx + uuvx);
  r.y = a.y + ty + 2.0f * (aw * uvy + uuvy);
  r.z = a.z + tz + 2.0f * (aw * uvz + uuvz);
  return r;
}

inline CPose pose_inverse(CPose p) {
  // conjugate rotation
  CPose inv{};
  inv.qx = -p.qx;
  inv.qy = -p.qy;
  inv.qz = -p.qz;
  inv.qw = p.qw;
  // -R^T * t == rotate -t by the conjugate quaternion
  float ax = inv.qx, ay = inv.qy, az = inv.qz, aw = inv.qw;
  float tx = -p.x, ty = -p.y, tz = -p.z;
  float uvx = ay * tz - az * ty, uvy = az * tx - ax * tz, uvz = ax * ty - ay * tx;
  float uuvx = ay * uvz - az * uvy, uuvy = az * uvx - ax * uvz, uuvz = ax * uvy - ay * uvx;
  inv.x = tx + 2.0f * (aw * uvx + uuvx);
  inv.y = ty + 2.0f * (aw * uvy + uuvy);
  inv.z = tz + 2.0f * (aw * uvz + uuvz);
  return inv;
}

// Object-centric reachable-base construction for the `mobile` domain,
// mirroring `common/streams_mobile.py`'s `sample_reachable_base` exactly
// (see that file's comment for why: inverts hop-mobile's own
// `base_too_far` reachability relation instead of blindly sampling +
// rejecting, verified there to fix a real quadratic move-base blowup).
// Deliberately single-shot (no internal "keep sampling until in bounds"
// loop) for the same reason `hopcxx_*_ik` is single-shot now -- the
// completion tree's own resampling is what should pay for a rejected draw,
// not a hidden loop (this one has a high, cheap accept rate -- ~92%, per
// project memory -- so the effect is small, but the principle should still
// be applied uniformly). Returns false if the constructed base pose falls
// outside `baseBounds` ([xmin, ymin, xmax, ymax]).
inline bool sample_reachable_base(CPose p, const float* baseBounds, CPose* out) {
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  float r = std::sqrt(unit(rng));
  float phi = -unit(rng) * (float)M_PI / 2.0f;  // uniform(-pi/2, 0)
  float x = r * std::cos(phi), y = r * std::sin(phi);
  float yaw = unit(rng) * 2.0f * (float)M_PI;
  CPose rel = pose_from_xyz_yaw(x, y, p.z, yaw);
  CPose bq = pose_mul(p, pose_inverse(rel));
  if (bq.x < baseBounds[0] || bq.x > baseBounds[2] || bq.y < baseBounds[1] || bq.y > baseBounds[3]) {
    return false;
  }
  *out = bq;
  return true;
}
