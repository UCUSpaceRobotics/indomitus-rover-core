// Tests for the SlopeLayer geometry.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "nav2_costmap_2d/cost_values.hpp"
#include "rover_costmap_plugins/slope_math.hpp"

using rover_costmap_plugins::slope_math::CellMoments;
using rover_costmap_plugins::slope_math::PlaneFit;
using rover_costmap_plugins::slope_math::baseToWorld;
using rover_costmap_plugins::slope_math::effectiveRoughnessThreshold;
using rover_costmap_plugins::slope_math::fitPlane;
using rover_costmap_plugins::slope_math::roughnessToSlopeDeg;
using rover_costmap_plugins::slope_math::sensorIsStale;
using rover_costmap_plugins::slope_math::slopeDegFromNormal;
using rover_costmap_plugins::slope_math::slopeToCost;
using rover_costmap_plugins::slope_math::worldCell;
using rover_costmap_plugins::slope_math::worldToBase;

namespace
{
constexpr double kMinSpread = 1.0e-4;
constexpr int kMinPoints = 7;

// Ground tilted by slope_deg about the y axis, sampled on a grid.
// noise adds bumps to stand in for a rough surface.
CellMoments makeSlopedPatch(double slope_deg, double extent = 0.25, int n = 5, double noise = 0.0)
{
  CellMoments m;
  const double k = std::tan(slope_deg * M_PI / 180.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double x = -extent / 2.0 + extent * i / (n - 1);
      const double y = -extent / 2.0 + extent * j / (n - 1);
      const double bump = noise * (((i + j) % 2 == 0) ? 1.0 : -1.0);
      m.add(x, y, k * x + bump);
    }
  }
  return m;
}
}  // namespace

// --- plane fitting -----------------------------------------------------------

TEST(SlopeMath, FlatGroundFitsZeroSlope)
{
  const CellMoments m = makeSlopedPatch(0.0);
  const PlaneFit fit = fitPlane(m, kMinPoints, kMinSpread);

  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(std::abs(fit.nz), 1.0, 1e-6);
  EXPECT_NEAR(fit.residual, 0.0, 1e-9);
  EXPECT_NEAR(slopeDegFromNormal(fit.nx, fit.ny, fit.nz, 1.0, 0.0, 0.0, 0.0), 0.0, 1e-6);
}

TEST(SlopeMath, SlopedGroundRecoversTheAngle)
{
  for (const double truth : {5.0, 15.0, 30.0, 45.0}) {
    const CellMoments m = makeSlopedPatch(truth);
    const PlaneFit fit = fitPlane(m, kMinPoints, kMinSpread);

    ASSERT_TRUE(fit.valid) << "slope " << truth;
    const double got = slopeDegFromNormal(fit.nx, fit.ny, fit.nz, 1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(got, truth, 1e-4) << "slope " << truth;
  }
}

TEST(SlopeMath, RoughSurfaceRaisesResidualNotSlope)
{
  const CellMoments smooth = makeSlopedPatch(0.0, 0.25, 5, 0.0);
  const CellMoments rough = makeSlopedPatch(0.0, 0.25, 5, 0.05);

  const PlaneFit smooth_fit = fitPlane(smooth, kMinPoints, kMinSpread);
  const PlaneFit rough_fit = fitPlane(rough, kMinPoints, kMinSpread);

  ASSERT_TRUE(smooth_fit.valid);
  ASSERT_TRUE(rough_fit.valid);
  EXPECT_GT(rough_fit.residual, smooth_fit.residual);
  // The bumps do not cancel exactly, so this lands just under their size.
  EXPECT_NEAR(rough_fit.residual, 0.05, 1e-3);
  // The average plane is flat, so roughness must not show up as slope.
  EXPECT_NEAR(
    slopeDegFromNormal(rough_fit.nx, rough_fit.ny, rough_fit.nz, 1.0, 0.0, 0.0, 0.0), 0.0, 1e-6);
}

// --- degenerate input rejection ---------------------------------------------

TEST(SlopeMath, CollinearPointsAreRejected)
{
  // Points on one line define no plane, so the normal could point anywhere.
  CellMoments m;
  for (int i = 0; i < 10; ++i) {
    m.add(0.01 * i, 0.0, 0.0);
  }
  EXPECT_FALSE(fitPlane(m, kMinPoints, kMinSpread).valid);
}

TEST(SlopeMath, DuplicatePointsAreRejected)
{
  CellMoments m;
  for (int i = 0; i < 20; ++i) {
    m.add(0.5, 0.5, 0.25);
  }
  EXPECT_FALSE(fitPlane(m, kMinPoints, kMinSpread).valid);
}

TEST(SlopeMath, TooFewPointsAreRejected)
{
  CellMoments m;
  m.add(0.0, 0.0, 0.0);
  m.add(0.1, 0.0, 0.0);
  m.add(0.0, 0.1, 0.0);
  EXPECT_FALSE(fitPlane(m, kMinPoints, kMinSpread).valid);
  // The same points pass once the minimum is lowered.
  EXPECT_TRUE(fitPlane(m, 3, kMinSpread).valid);
}

TEST(SlopeMath, NonFiniteMomentsAreRejected)
{
  CellMoments m = makeSlopedPatch(10.0);
  m.sxx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fitPlane(m, kMinPoints, kMinSpread).valid);
}

// --- gravity referencing -----------------------------------------------------

TEST(SlopeMath, FlatGroundStaysFlatWhenTheVehicleIsTilted)
{
  // The rover pitches 20 deg on flat ground. Rotating into the costmap frame
  // must cancel that tilt exactly.
  const double pitch = 20.0 * M_PI / 180.0;
  const double nx = std::sin(pitch), ny = 0.0, nz = std::cos(pitch);

  // Undoes the pitch: a rotation of -pitch about y.
  const double half = -pitch / 2.0;
  const double qw = std::cos(half), qy = std::sin(half);

  EXPECT_NEAR(slopeDegFromNormal(nx, ny, nz, qw, 0.0, qy, 0.0), 0.0, 1e-6);
  // Uncorrected, the same reading looks like a 20 deg slope.
  EXPECT_NEAR(slopeDegFromNormal(nx, ny, nz, 1.0, 0.0, 0.0, 0.0), 20.0, 1e-6);
}

TEST(SlopeMath, NormalSignIsIrrelevant)
{
  // The fit may return either sign of normal, both must give one slope.
  EXPECT_NEAR(
    slopeDegFromNormal(0.5, 0.0, 0.866025, 1.0, 0.0, 0.0, 0.0),
    slopeDegFromNormal(-0.5, 0.0, -0.866025, 1.0, 0.0, 0.0, 0.0),
    1e-9);
}

// --- cost banding ------------------------------------------------------------

TEST(SlopeMath, CostBandEndpointsAndGrading)
{
  const double traversable = 18.0, lethal = 30.0;

  EXPECT_EQ(slopeToCost(0.0, traversable, lethal), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(slopeToCost(traversable, traversable, lethal), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(slopeToCost(lethal, traversable, lethal), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(slopeToCost(89.0, traversable, lethal), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Between the thresholds the cost is graded, never lethal.
  const unsigned char mid = slopeToCost(24.0, traversable, lethal);
  EXPECT_GT(mid, nav2_costmap_2d::FREE_SPACE);
  EXPECT_LT(mid, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Monotonic in slope.
  EXPECT_LT(slopeToCost(20.0, traversable, lethal), slopeToCost(28.0, traversable, lethal));
}

TEST(SlopeMath, RoughnessIsCappedBelowLethalUnlessAllowed)
{
  const double traversable = 18.0, lethal = 30.0, thresh = 0.04, sat = 4.0;
  const double very_rough = thresh * 100.0;

  const double capped =
    roughnessToSlopeDeg(very_rough, thresh, traversable, lethal, sat, /*allow_lethal=*/false);
  EXPECT_LT(capped, lethal);
  EXPECT_LT(slopeToCost(capped, traversable, lethal), nav2_costmap_2d::LETHAL_OBSTACLE);

  const double uncapped =
    roughnessToSlopeDeg(very_rough, thresh, traversable, lethal, sat, /*allow_lethal=*/true);
  EXPECT_GE(uncapped, lethal);

  // Roughness under the threshold adds nothing.
  EXPECT_DOUBLE_EQ(
    roughnessToSlopeDeg(thresh * 0.5, thresh, traversable, lethal, sat, true), traversable);
}

TEST(SlopeMath, RoughnessThresholdWidensWithRangeAndSparsity)
{
  const double base = 0.045, coeff = 0.015;
  const int dense = kMinPoints * 5;

  EXPECT_GT(
    effectiveRoughnessThreshold(base, coeff, 4.0, dense, kMinPoints),
    effectiveRoughnessThreshold(base, coeff, 0.0, dense, kMinPoints));

  // A cell with few points is trusted less than a dense one at equal range.
  EXPECT_GT(
    effectiveRoughnessThreshold(base, coeff, 2.0, kMinPoints, kMinPoints),
    effectiveRoughnessThreshold(base, coeff, 2.0, dense, kMinPoints));
}

// --- persistence geometry ----------------------------------------------------

TEST(SlopeMath, BaseWorldRoundTrip)
{
  const double yaw = 0.7, tx = 1.5, ty = -2.25;
  const double c = std::cos(yaw), s = std::sin(yaw);

  double wx, wy, bx, by;
  baseToWorld(3.0, -1.0, tx, ty, c, s, wx, wy);
  worldToBase(wx, wy, tx, ty, c, s, bx, by);

  EXPECT_NEAR(bx, 3.0, 1e-12);
  EXPECT_NEAR(by, -1.0, 1e-12);
}

TEST(SlopeMath, WorldCellIndexingAndBoundsRejection)
{
  const double origin = -50.0, res = 0.25;
  const int w = 400, h = 400;  // 100 m across
  int gx = -1, gy = -1;

  ASSERT_TRUE(worldCell(0.0, 0.0, origin, origin, res, w, h, gx, gy));
  EXPECT_EQ(gx, 200);
  EXPECT_EQ(gy, 200);

  // Negative coordinates must floor. Truncating folds two cells into one.
  int gx_a = 0, gy_a = 0, gx_b = 0, gy_b = 0;
  ASSERT_TRUE(worldCell(-0.1, 0.0, origin, origin, res, w, h, gx_a, gy_a));
  ASSERT_TRUE(worldCell(0.1, 0.0, origin, origin, res, w, h, gx_b, gy_b));
  EXPECT_NE(gx_a, gx_b);

  // Points outside the grid are reported, not wrapped onto a valid cell.
  EXPECT_FALSE(worldCell(60.0, 0.0, origin, origin, res, w, h, gx, gy));
  EXPECT_FALSE(worldCell(0.0, -60.0, origin, origin, res, w, h, gx, gy));
}

TEST(SlopeMath, ObservationFromDifferentPosesLandsOnTheSameWorldCell)
{
  // One spot seen from two rover poses must land on one memory cell,
  // otherwise the terrain memory cannot line up.
  const double origin = -50.0, res = 0.25;
  const int w = 400, h = 400;

  // Spot at world (2.0, 3.0). Pose A: at origin, yaw 0.
  double yaw = 0.0, c = std::cos(yaw), s = std::sin(yaw);
  double bx, by, wx, wy;
  worldToBase(2.0, 3.0, 0.0, 0.0, c, s, bx, by);
  baseToWorld(bx, by, 0.0, 0.0, c, s, wx, wy);
  int gx_a = 0, gy_a = 0;
  ASSERT_TRUE(worldCell(wx, wy, origin, origin, res, w, h, gx_a, gy_a));

  // Pose B: rover at (1,1) turned by 0.9 rad. Its transform is t = -R(yaw) p.
  yaw = 0.9; c = std::cos(yaw); s = std::sin(yaw);
  const double rx = 1.0, ry = 1.0;
  const double tx = -(c * rx + s * ry);
  const double ty = -(-s * rx + c * ry);
  worldToBase(2.0, 3.0, tx, ty, c, s, bx, by);
  baseToWorld(bx, by, tx, ty, c, s, wx, wy);
  int gx_b = 0, gy_b = 0;
  ASSERT_TRUE(worldCell(wx, wy, origin, origin, res, w, h, gx_b, gy_b));

  EXPECT_EQ(gx_a, gx_b);
  EXPECT_EQ(gy_a, gy_b);
}

// --- sensor staleness --------------------------------------------------------

TEST(SlopeMath, StaleSensorDetection)
{
  EXPECT_FALSE(sensorIsStale(0.1, 2.0));
  EXPECT_FALSE(sensorIsStale(2.0, 2.0));
  EXPECT_TRUE(sensorIsStale(2.5, 2.0));
  // Non-positive timeout disables the check.
  EXPECT_FALSE(sensorIsStale(1000.0, 0.0));
  EXPECT_FALSE(sensorIsStale(1000.0, -1.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
