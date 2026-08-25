#pragma once
#ifndef ROVER_LOCALIZATION__TILT_MATH_HPP_
#define ROVER_LOCALIZATION__TILT_MATH_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Pure attitude math for tilt_broadcaster_node. No ROS state, so it is
// unit testable.
namespace rover_localization
{
namespace tilt_math
{

// Body orientation in the world. q_mount is how the IMU sits on the body;
// removing it leaves the body's own orientation.
inline tf2::Quaternion bodyOrientationFromImu(
  const tf2::Quaternion & q_imu_raw, const tf2::Quaternion & q_mount)
{
  return q_imu_raw * q_mount.inverse();
}

// Roll and pitch only. The EKF owns yaw, so keeping it here would count
// heading twice and drift the tree.
inline tf2::Quaternion tiltOnly(const tf2::Quaternion & q_world_body)
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q_world_body).getRPY(roll, pitch, yaw);
  tf2::Quaternion q_tilt;
  q_tilt.setRPY(roll, pitch, 0.0);
  return q_tilt;
}

// Raw IMU reading and mount offset to a yaw-free tilt.
inline tf2::Quaternion tiltFromImu(
  const tf2::Quaternion & q_imu_raw, const tf2::Quaternion & q_mount)
{
  return tiltOnly(bodyOrientationFromImu(q_imu_raw, q_mount));
}

// True when a reading is too old to trust. Zero or less disables the check.
inline bool isStale(double age_s, double timeout_s)
{
  if (timeout_s <= 0.0) {return false;}
  return age_s > timeout_s;
}

}  // namespace tilt_math
}  // namespace rover_localization

#endif  // ROVER_LOCALIZATION__TILT_MATH_HPP_
