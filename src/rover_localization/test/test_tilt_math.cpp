// Tests for the tilt broadcaster's attitude math.

#include <cmath>

#include <gtest/gtest.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rover_localization/tilt_math.hpp"

using rover_localization::tilt_math::bodyOrientationFromImu;
using rover_localization::tilt_math::isStale;
using rover_localization::tilt_math::tiltFromImu;
using rover_localization::tilt_math::tiltOnly;

namespace
{
constexpr double kDeg = M_PI / 180.0;

tf2::Quaternion rpy(double r, double p, double y)
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  return q;
}

void getRPY(const tf2::Quaternion & q, double & r, double & p, double & y)
{
  tf2::Matrix3x3(q).getRPY(r, p, y);
}
}  // namespace

TEST(TiltMath, YawIsAlwaysZero)
{
  // base_footprint must stay yaw-free, since the EKF already owns heading.
  for (const double yaw_in : {-2.0, -0.5, 0.0, 0.5, 2.0, 3.0}) {
    const tf2::Quaternion q = tiltOnly(rpy(10.0 * kDeg, -7.0 * kDeg, yaw_in));
    double r, p, y;
    getRPY(q, r, p, y);
    EXPECT_NEAR(y, 0.0, 1e-9) << "yaw in " << yaw_in;
    EXPECT_NEAR(r, 10.0 * kDeg, 1e-9);
    EXPECT_NEAR(p, -7.0 * kDeg, 1e-9);
  }
}

TEST(TiltMath, MountOffsetIsRemoved)
{
  // The IMU sits pitched down 15 deg. On a level rover the raw reading shows
  // that 15 deg, but the body must read flat.
  const tf2::Quaternion q_mount = rpy(0.0, 15.0 * kDeg, 0.0);
  const tf2::Quaternion q_body_truth = rpy(0.0, 0.0, 0.0);
  const tf2::Quaternion q_imu_raw = q_body_truth * q_mount;

  double r, p, y;
  getRPY(bodyOrientationFromImu(q_imu_raw, q_mount), r, p, y);
  EXPECT_NEAR(r, 0.0, 1e-9);
  EXPECT_NEAR(p, 0.0, 1e-9);
}

TEST(TiltMath, RealTiltSurvivesMountRemoval)
{
  // Same mount, but the rover really is pitched 12 deg and rolled 5 deg.
  const tf2::Quaternion q_mount = rpy(0.0, 15.0 * kDeg, 0.0);
  const tf2::Quaternion q_body_truth = rpy(5.0 * kDeg, -12.0 * kDeg, 0.0);
  const tf2::Quaternion q_imu_raw = q_body_truth * q_mount;

  double r, p, y;
  getRPY(tiltFromImu(q_imu_raw, q_mount), r, p, y);
  EXPECT_NEAR(r, 5.0 * kDeg, 1e-9);
  EXPECT_NEAR(p, -12.0 * kDeg, 1e-9);
  EXPECT_NEAR(y, 0.0, 1e-9);
}

TEST(TiltMath, HeadingDoesNotLeakThroughMountRemoval)
{
  // Driving on a 40 deg heading over ground pitched 8 deg. The tilt must
  // survive and the heading must not.
  const tf2::Quaternion q_mount = rpy(0.0, 15.0 * kDeg, 0.0);
  const tf2::Quaternion q_body_truth = rpy(0.0, 8.0 * kDeg, 40.0 * kDeg);
  const tf2::Quaternion q_imu_raw = q_body_truth * q_mount;

  double r, p, y;
  getRPY(tiltFromImu(q_imu_raw, q_mount), r, p, y);
  EXPECT_NEAR(r, 0.0, 1e-9);
  EXPECT_NEAR(p, 8.0 * kDeg, 1e-9);
  EXPECT_NEAR(y, 0.0, 1e-9);
}

TEST(TiltMath, IdentityMountIsANoOp)
{
  const tf2::Quaternion q_mount(0.0, 0.0, 0.0, 1.0);
  const tf2::Quaternion q_body = rpy(3.0 * kDeg, -4.0 * kDeg, 0.0);

  double r, p, y;
  getRPY(tiltFromImu(q_body, q_mount), r, p, y);
  EXPECT_NEAR(r, 3.0 * kDeg, 1e-9);
  EXPECT_NEAR(p, -4.0 * kDeg, 1e-9);
}

TEST(TiltMath, StaleImuDetection)
{
  // A frozen IMU must not be sent on as a fresh transform.
  EXPECT_FALSE(isStale(0.1, 0.5));
  EXPECT_FALSE(isStale(0.5, 0.5));
  EXPECT_TRUE(isStale(0.6, 0.5));
  EXPECT_TRUE(isStale(30.0, 0.5));
  // Non-positive timeout disables the check.
  EXPECT_FALSE(isStale(1000.0, 0.0));
  EXPECT_FALSE(isStale(1000.0, -1.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
