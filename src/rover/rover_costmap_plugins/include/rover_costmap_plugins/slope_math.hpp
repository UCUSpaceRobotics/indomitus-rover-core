#pragma once
#ifndef ROVER_COSTMAP_PLUGINS__SLOPE_MATH_HPP_
#define ROVER_COSTMAP_PLUGINS__SLOPE_MATH_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

#include "nav2_costmap_2d/cost_values.hpp"

// Pure geometry for SlopeLayer. No ROS state, so it is unit testable.
namespace rover_costmap_plugins
{
namespace slope_math
{

// Sums needed to fit a plane over the points in one grid cell.
struct CellMoments
{
  int count = 0;
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double sxx = 0.0, syy = 0.0, szz = 0.0;
  double sxy = 0.0, sxz = 0.0, syz = 0.0;

  void add(double x, double y, double z)
  {
    count++;
    sum_x += x; sum_y += y; sum_z += z;
    sxx += x * x; syy += y * y; szz += z * z;
    sxy += x * y; sxz += x * z; syz += y * z;
  }
};

// Fitted plane for one cell. valid is false when the points do not
// define a plane: too few, collinear, coincident, or non-finite.
struct PlaneFit
{
  double nx = 0.0, ny = 0.0, nz = 1.0;
  double residual = 0.0;  // std-dev of the points about the plane, meters
  double mean_z = 0.0;
  bool valid = false;
};

// Rotate vector (x,y,z) by unit quaternion (qw,qx,qy,qz): v' = q*v*q^-1.
inline void rotateByQuat(
  double qw, double qx, double qy, double qz,
  double x, double y, double z,
  double & ox, double & oy, double & oz)
{
  const double tx = 2.0 * (qy * z - qz * y);
  const double ty = 2.0 * (qz * x - qx * z);
  const double tz = 2.0 * (qx * y - qy * x);
  ox = x + qw * tx + (qy * tz - qz * ty);
  oy = y + qw * ty + (qz * tx - qx * tz);
  oz = z + qw * tz + (qx * ty - qy * tx);
}

// Fits a plane to the cell. Eigenvalues are ascending: the smallest is the
// spread across the plane (roughness) and its vector is the plane normal.
inline PlaneFit fitPlane(
  const CellMoments & m, int min_points, double min_plane_spread)
{
  PlaneFit fit;
  if (m.count < min_points || m.count < 3) {return fit;}

  // Must run before the clamp below, which hides a NaN as 0.0.
  if (!std::isfinite(m.sum_x) || !std::isfinite(m.sum_y) || !std::isfinite(m.sum_z) ||
      !std::isfinite(m.sxx) || !std::isfinite(m.syy) || !std::isfinite(m.szz) ||
      !std::isfinite(m.sxy) || !std::isfinite(m.sxz) || !std::isfinite(m.syz))
  {
    return fit;
  }

  const double n = static_cast<double>(m.count);
  const double mx = m.sum_x / n, my = m.sum_y / n, mz = m.sum_z / n;

  // Rounding can push a variance slightly below zero, which breaks the solver.
  const double cxx = std::max(0.0, m.sxx / n - mx * mx);
  const double cyy = std::max(0.0, m.syy / n - my * my);
  const double czz = std::max(0.0, m.szz / n - mz * mz);
  const double cxy = m.sxy / n - mx * my;
  const double cxz = m.sxz / n - mx * mz;
  const double cyz = m.syz / n - my * mz;

  if (!std::isfinite(cxx) || !std::isfinite(cyy) || !std::isfinite(czz) ||
      !std::isfinite(cxy) || !std::isfinite(cxz) || !std::isfinite(cyz))
  {
    return fit;
  }

  Eigen::Matrix3d cov;
  cov << cxx, cxy, cxz,
         cxy, cyy, cyz,
         cxz, cyz, czz;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(cov);
  if (solver.info() != Eigen::Success) {return fit;}

  const Eigen::Vector3d ev = solver.eigenvalues();
  if (!ev.allFinite()) {return fit;}
  // Points lie on a line or one spot, so the normal would point anywhere.
  if (ev(1) < min_plane_spread) {return fit;}

  const Eigen::Vector3d normal = solver.eigenvectors().col(0);
  if (!normal.allFinite()) {return fit;}

  fit.nx = normal.x();
  fit.ny = normal.y();
  fit.nz = normal.z();
  fit.residual = std::sqrt(std::max(0.0, ev(0)));
  fit.mean_z = mz;
  fit.valid = true;
  return fit;
}

// Tilt of the plane against gravity. The quaternion turns the normal from the
// base frame into the costmap frame, so rover tilt does not count as slope.
inline double slopeDegFromNormal(
  double nx, double ny, double nz,
  double qw, double qx, double qy, double qz)
{
  double gx, gy, gz;
  rotateByQuat(qw, qx, qy, qz, nx, ny, nz, gx, gy, gz);
  if (gz < 0.0) {gz = -gz;}  // the normal may point down, take the up one
  const double slope_rad = std::acos(std::clamp(gz, -1.0, 1.0));
  return slope_rad * 180.0 / M_PI;
}

// Free below the traversable angle, lethal above the lethal angle,
// graded in between.
inline unsigned char slopeToCost(
  double slope_deg, double traversable_deg, double lethal_deg)
{
  if (slope_deg >= lethal_deg) {return nav2_costmap_2d::LETHAL_OBSTACLE;}
  if (slope_deg <= traversable_deg) {return nav2_costmap_2d::FREE_SPACE;}
  const double span = lethal_deg - traversable_deg;
  if (span <= 0.0) {return nav2_costmap_2d::LETHAL_OBSTACLE;}
  const double t = (slope_deg - traversable_deg) / span;
  return static_cast<unsigned char>(t * (nav2_costmap_2d::LETHAL_OBSTACLE - 1));
}

// Allowed roughness. Grows with distance and with fewer points, because both
// make the fit less reliable.
inline double effectiveRoughnessThreshold(
  double base_thresh, double range_coeff, double range_m,
  int point_count, int min_points_per_cell)
{
  double thresh = base_thresh + range_coeff * range_m;
  const int sparsity_span = min_points_per_cell * 2;
  if (sparsity_span > 0 && point_count < min_points_per_cell + sparsity_span) {
    const double sparsity_factor =
      static_cast<double>(min_points_per_cell + sparsity_span - point_count) / sparsity_span;
    thresh *= (1.0 + sparsity_factor);
  }
  return thresh;
}

// Turns roughness into an angle, so it shares one cost band with real slope.
inline double roughnessToSlopeDeg(
  double residual, double effective_thresh,
  double traversable_deg, double lethal_deg,
  double saturation_mult, bool allow_lethal)
{
  if (residual <= effective_thresh || effective_thresh <= 0.0) {return traversable_deg;}
  const double span = lethal_deg - traversable_deg;
  const double excess_ratio = (residual - effective_thresh) / effective_thresh;
  double slope = traversable_deg +
    std::min(excess_ratio, saturation_mult) * span / saturation_mult;
  if (!allow_lethal) {
    slope = std::min(slope, traversable_deg + 0.98 * span);
  }
  return slope;
}

// Base frame to costmap frame. (tx, ty, yaw) is the costmap -> base transform,
// so this applies its inverse.
inline void baseToWorld(
  double bx, double by, double tx, double ty, double cos_yaw, double sin_yaw,
  double & wx, double & wy)
{
  const double dx = bx - tx, dy = by - ty;
  wx = cos_yaw * dx + sin_yaw * dy;
  wy = -sin_yaw * dx + cos_yaw * dy;
}

// Costmap frame to base frame.
inline void worldToBase(
  double wx, double wy, double tx, double ty, double cos_yaw, double sin_yaw,
  double & bx, double & by)
{
  bx = cos_yaw * wx - sin_yaw * wy + tx;
  by = sin_yaw * wx + cos_yaw * wy + ty;
}

// Cell holding this point. False when it lies outside the grid, so the caller
// drops it instead of writing to a wrong cell.
inline bool worldCell(
  double wx, double wy, double origin_x, double origin_y, double resolution,
  int width, int height, int & gx, int & gy)
{
  if (resolution <= 0.0) {return false;}
  gx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  gy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return gx >= 0 && gx < width && gy >= 0 && gy < height;
}

// True when a reading is too old to trust. Zero or less disables the check.
inline bool sensorIsStale(double age_s, double timeout_s)
{
  if (timeout_s <= 0.0) {return false;}
  return age_s > timeout_s;
}

}  // namespace slope_math
}  // namespace rover_costmap_plugins

#endif  // ROVER_COSTMAP_PLUGINS__SLOPE_MATH_HPP_
