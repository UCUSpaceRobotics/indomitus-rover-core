// Speed profile of SCurveLimiter: trapezoidal without jerk, S-curve with it.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "rover_controller/s_curve_limiter.hpp"

using rover_controller::SCurveLimiter;

namespace {

constexpr double kDt = 0.01;  // 100 Hz, the controller_manager rate

struct Profile {
    double max_accel{0.0};      ///< largest |acceleration| seen
    double max_jerk{0.0};       ///< largest |d(accel)/dt| seen
    double overshoot{0.0};      ///< furthest past the target
    double settle_time{-1.0};   ///< first time value == target exactly
};

Profile run(SCurveLimiter & lim, double target, double duration) {
    Profile p;
    double v_prev = lim.current();
    double a_prev = 0.0;
    const double start = lim.current();
    for (int k = 1; k * kDt <= duration + 1e-9; ++k) {
        const double v = lim.update(target, kDt);
        const double a = (v - v_prev) / kDt;
        p.max_accel = std::max(p.max_accel, std::abs(a));
        p.max_jerk  = std::max(p.max_jerk, std::abs(a - a_prev) / kDt);
        p.overshoot = std::max(p.overshoot, (v - target) * (target > start ? 1.0 : -1.0));
        if (p.settle_time < 0.0 && v == target) { p.settle_time = k * kDt; }
        v_prev = v;
        a_prev = a;
    }
    return p;
}

}  // namespace

TEST(SCurveLimiter, WithoutJerkIsTrapezoidal) {
    SCurveLimiter lim{0.6, 1.2};
    const Profile p = run(lim, 1.0, 3.0);
    EXPECT_NEAR(p.max_accel, 0.6, 1e-9);
    // Acceleration jumps 0 → 0.6 within one cycle.
    EXPECT_NEAR(p.max_jerk, 0.6 / kDt, 1e-6);
    EXPECT_NEAR(p.settle_time, 1.0 / 0.6, kDt + 1e-9);
}

TEST(SCurveLimiter, AccelerationIsSCurve) {
    SCurveLimiter lim{0.6, 1.2, 2.0, 6.0};
    const Profile p = run(lim, 1.0, 4.0);
    EXPECT_LE(p.max_accel, 0.6 + 1e-9);
    // Jerk stays near its limit; the final snap onto the target adds at most
    // one cycle's worth of leftover acceleration.
    EXPECT_LT(p.max_jerk, 2.0 * 2.5);
    EXPECT_LE(p.overshoot, 1e-9);
    // Trapezoid time 1/0.6 plus one accel/jerk ramp (0.3 s), small tracking lag.
    EXPECT_GT(p.settle_time, 1.0 / 0.6 + 0.2);
    EXPECT_LT(p.settle_time, 1.0 / 0.6 + 0.6);
}

TEST(SCurveLimiter, BrakingUsesDecelLimits) {
    SCurveLimiter lim{0.6, 1.2, 2.0, 6.0};
    lim.reset(1.0);
    const Profile p = run(lim, 0.0, 3.0);
    EXPECT_LE(p.max_accel, 1.2 + 1e-9);
    EXPECT_GT(p.max_accel, 1.1);          // cruises at max_decel
    EXPECT_LT(p.max_jerk, 6.0 * 2.5);
    EXPECT_LE(p.overshoot, 1e-9);         // never reverses past zero
    EXPECT_LT(p.settle_time, 1.0 / 1.2 + 0.4);
}

TEST(SCurveLimiter, ShortStepNeverReachesMaxAccel) {
    // 0.05 m/s needs less than a full ramp in and out: triangular accel.
    SCurveLimiter lim{0.6, 1.2, 2.0, 6.0};
    const Profile p = run(lim, 0.05, 2.0);
    EXPECT_LT(p.max_accel, 0.6);
    EXPECT_LE(p.overshoot, 1e-9);
    EXPECT_GT(p.settle_time, 0.0);
}

TEST(SCurveLimiter, TargetChangeMidRampKeepsAccelerationContinuous) {
    SCurveLimiter lim{0.6, 1.2, 2.0, 6.0};
    run(lim, 1.0, 0.8);                  // mid-ramp, accelerating
    const double a_before = lim.acceleration();
    ASSERT_GT(a_before, 0.3);
    lim.update(0.0, kDt);                // driver lets go of the stick
    // Acceleration may only move by decel_jerk·dt, not flip at once.
    EXPECT_NEAR(lim.acceleration(), a_before - 6.0 * kDt, 1e-9);
    const Profile p = run(lim, 0.0, 3.0);
    EXPECT_EQ(lim.current(), 0.0);
    EXPECT_LE(p.overshoot, 1e-9);
}

TEST(SCurveLimiter, ResetClearsAcceleration) {
    SCurveLimiter lim{0.6, 1.2, 2.0, 6.0};
    run(lim, 1.0, 0.5);
    lim.reset(0.0);
    EXPECT_EQ(lim.current(), 0.0);
    EXPECT_EQ(lim.acceleration(), 0.0);
}
