#pragma once

#include <algorithm>
#include <cmath>

#include "rover_controller/swerve_kinematics.hpp"  // clamp

namespace rover_controller {

/**
 * Rate limiter toward a moving target, optionally jerk-limited.
 *
 * With jerk 0 it behaves like SlewRateLimiter: the value moves at a constant
 * rate, a trapezoidal profile whose acceleration jumps straight to max_accel
 * and back. With a positive jerk the acceleration itself ramps, giving an
 * S-curve: no step in acceleration at the start or end of a ramp, at the cost
 * of a(max)/jerk extra seconds per ramp (stops included).
 *
 * Braking (target closer to zero, or a sign flip) uses the decel limits.
 */
class SCurveLimiter {
public:
    SCurveLimiter(
        double max_accel,
        double max_decel,
        double accel_jerk = 0.0,
        double decel_jerk = 0.0)
    : max_accel_(max_accel),
        max_decel_(max_decel),
        accel_jerk_(accel_jerk),
        decel_jerk_(decel_jerk)
    {}

    double update(double target, double dt) {
        const double diff    = target - current_;
        const bool   braking = (std::abs(target) < std::abs(current_))
                              || (current_ * target < 0.0);
        const double rate    = braking ? max_decel_ : max_accel_;
        const double jerk    = braking ? decel_jerk_ : accel_jerk_;

        if (jerk <= 0.0 || dt <= 0.0) {
            const double step = clamp(diff, -rate * dt, rate * dt);
            current_ += step;
            accel_    = (dt > 0.0) ? step / dt : 0.0;
            return current_;
        }

        // Acceleration that, ramped down to zero at `jerk`, lands exactly on
        // the target: a²/(2·jerk) = what is left of diff after this cycle's
        // own step, (a_prev + a)/2·dt. Tracking it (capped by `rate`) ramps
        // up, cruises and ramps down without a separate phase machine, and
        // follows a target that keeps moving. Taking this cycle's step into
        // account is what keeps it from arriving with acceleration left over.
        // Worked in the direction of diff, so the root is the positive one.
        const double a_prev = accel_;
        const double jdt    = jerk * dt;

        // Land when this cycle can finish the move exactly with no more than a
        // jerk step on either side. Without it the discrete loop ends in a
        // ±jerk·dt limit cycle a hair short of the target.
        const double a_exact = 2.0 * diff / dt - a_prev;
        if (std::abs(a_exact - a_prev) <= jdt && std::abs(a_exact) <= jdt) {
            current_ = target;
            accel_   = 0.0;
            return current_;
        }

        const double sgn    = (diff >= 0.0) ? 1.0 : -1.0;
        const double e      = std::abs(diff) - 0.5 * sgn * a_prev * dt;
        const double a_land = sgn * 0.5 *
            (-jdt + std::sqrt(std::max(0.0, jdt * jdt + 8.0 * jerk * e)));
        const double a_want = clamp(a_land, -rate, rate);
        accel_ += clamp(a_want - accel_, -jdt, jdt);

        const double next = current_ + 0.5 * (a_prev + accel_) * dt;
        if (diff != 0.0 && (target - next) * diff <= 0.0) {
            // Reached or stepped over the target: discretisation only, the
            // leftover acceleration is at most ~jerk·dt here.
            current_ = target;
            accel_   = 0.0;
        } else {
            current_ = next;
        }
        return current_;
    }

    double current() const { return current_; }
    double acceleration() const { return accel_; }
    void   reset(double v = 0.0) { current_ = v; accel_ = 0.0; }

private:
    double max_accel_;
    double max_decel_;
    double accel_jerk_;
    double decel_jerk_;
    double current_{0.0};
    double accel_{0.0};
};

}  // namespace rover_controller
