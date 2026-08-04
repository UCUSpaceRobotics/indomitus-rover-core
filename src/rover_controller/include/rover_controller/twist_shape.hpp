#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// twist_shape.hpp
//
// Splits a chassis twist into "shape" (what the wheels point at) and
// "magnitude" (how fast they spin), so the two can be smoothed independently.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>

#include "rover_controller/swerve_kinematics.hpp"

namespace rover_controller {

/**
 * TwistShape — a chassis twist split into shape + magnitude.
 *
 * ik_general() is homogeneous of degree 1: scaling (vx, vy, wz) by c leaves
 * every steering angle untouched and scales every wheel speed by c. So a twist
 * carries exactly two independent pieces of information:
 *
 *   shape      → the four steering angles, and nothing else
 *   magnitude  → the four wheel speeds, and nothing else
 *
 * Smoothing those separately is what stops the wheels twitching when only the
 * speed changes: (theta, phi) sit perfectly still while m ramps. Smoothing
 * vx/vy/wz independently instead — as swerve_controller.cpp does — lets their
 * ratios wander during the ramp, and the ratios are precisely what the
 * steering angles are made of.
 *
 * The shape is a unit vector on a sphere, in spherical coordinates whose pole
 * is spin-in-place:
 *
 *   u = ( cos(phi)·cos(theta),  cos(phi)·sin(theta),  sin(phi) )
 *
 *   theta — crab direction  (the vx/vy proportion)
 *   phi   — turn tightness, phi = atan(d·kappa) for curvature kappa = 1/R
 *
 * phi is the whole trick. Curvature runs to infinity for spin-in-place, which
 * is why a plain (v, theta, kappa) basis cannot express it — v·kappa forces
 * wz→0 as v→0. atan squashes that infinity into phi = ±pi/2, an ordinary point
 * on the sphere needing no special case anywhere:
 *
 *   straight        kappa = 0      phi = 0
 *   turn of R = d   kappa = 1/d    phi = 45°
 *   spin in place   kappa = inf    phi = 90°
 *
 * A Twist of (0, 0, wz) simply lands on the pole. The transition into it is a
 * great-circle arc — phi sweeping 0 → 90° walks the instantaneous centre of
 * rotation continuously in from infinity to the rover centre.
 *
 * m is signed. u and −u give identical steering angles with opposite wheel
 * speeds, so reversing is a sign flip on m rather than a 180° sweep of theta:
 * the rover decelerates through zero and backs up along the same wheel
 * geometry instead of swinging its wheels around. resolve_antipode() picks
 * whichever of ±u is nearer the current shape and moves the sign onto m.
 *
 * Normalising to a unit vector also means the *scale* of the incoming twist is
 * irrelevant to the shape: a Twist of 1e-6 m/s recovers exactly the same
 * (theta, phi) as one of 1.0 m/s. That is what makes a near-zero Twist usable
 * as a pure "point the wheels here" command — see park_speed in the controller.
 */
struct TwistShape {
    double m{0.0};      ///< signed magnitude [m/s]
    double theta{0.0};  ///< crab direction [rad]
    double phi{0.0};    ///< turn tightness [rad], within [-pi/2, +pi/2]
};

/// Wrap an angle into [-pi, +pi].
inline double wrap_pi(double a)
{
    return std::remainder(a, 2.0 * M_PI);
}

/// Unit shape vector for (theta, phi). Magnitude is ignored.
inline void shape_unit(const TwistShape & s, double & ux, double & uy, double & uz)
{
    const double cos_phi = std::cos(s.phi);
    ux = cos_phi * std::cos(s.theta);
    uy = cos_phi * std::sin(s.theta);
    uz = std::sin(s.phi);
}

/**
 * Split a twist into shape + magnitude.
 *
 * @param d  Rotation scale length [m] — the lever arm that converts wz into a
 *           linear speed so all three components share units. With d set to
 *           the wheel half-diagonal, m is the ground speed of the corner
 *           wheels during a spin and the chassis speed during translation, so
 *           one accel limit means the same physical thing in both.
 *
 * Returns m = 0 with theta/phi zeroed when the twist is numerically empty; the
 * caller should keep its previous shape in that case rather than adopt this
 * one, so the wheels hold their angle while the speed ramps out.
 */
inline TwistShape decompose(double vx, double vy, double wz, double d)
{
    const double tz = d * wz;
    const double m  = std::sqrt(vx * vx + vy * vy + tz * tz);

    if (m < 1e-12) { return TwistShape{}; }

    // hypot() is non-negative, so phi lands in [-pi/2, +pi/2] by construction.
    return TwistShape{m, std::atan2(vy, vx), std::atan2(tz, std::hypot(vx, vy))};
}

/// Rebuild a twist from a shape.
inline void recompose(
    const TwistShape & s, double d,
    double & vx, double & vy, double & wz)
{
    double ux, uy, uz;
    shape_unit(s, ux, uy, uz);
    vx = s.m * ux;
    vy = s.m * uy;
    wz = s.m * uz / d;
}

/// cos of the angle between two shapes on the sphere.
inline double shape_dot(const TwistShape & a, const TwistShape & b)
{
    return std::cos(a.phi) * std::cos(b.phi) * std::cos(a.theta - b.theta)
         + std::sin(a.phi) * std::sin(b.phi);
}

/**
 * Flip `target` to its antipode if that is the nearer way to express the same
 * motion given where `reference` currently points.
 *
 * The antipode of (theta, phi) is (theta + pi, -phi) with the sign of m
 * flipped; it describes travelling the same path backwards, which needs the
 * same steering angles. Choosing the near one keeps the wheels still through a
 * forward/reverse change.
 */
inline TwistShape resolve_antipode(TwistShape target, const TwistShape & reference)
{
    if (shape_dot(target, reference) >= 0.0) { return target; }

    target.theta = wrap_pi(target.theta + M_PI);
    target.phi   = -target.phi;
    target.m     = -target.m;
    return target;
}

/**
 * ShapeSmoother — rate-limits the shape (theta, phi) on the sphere.
 *
 * Magnitude is deliberately *not* smoothed here; the caller runs it through
 * its own accel/decel limiter, which is the entire point of the split.
 *
 * Rates are applied in the sphere's own metric, ds² = dphi² + cos²(phi)·dtheta².
 * The cos(phi) factor on theta is not a fudge: without it the pole behaves like
 * gimbal lock, and exiting a spin would crawl toward the new heading. With it,
 * theta goes free exactly where it stops meaning anything.
 */
class ShapeSmoother {
public:
    ShapeSmoother(double max_theta_rate, double max_phi_rate)
    : max_theta_rate_(max_theta_rate),
      max_phi_rate_(max_phi_rate)
    {}

    /**
     * Advance the stored shape one step toward `target`.
     *
     * @return The smoothed shape. Its `.m` is the *target* magnitude with its
     *         sign resolved against the chosen hemisphere — not smoothed.
     */
    TwistShape step(TwistShape target, double dt)
    {
        target = resolve_antipode(target, current_);

        // theta is circular; take the short way round.
        const double d_theta = wrap_pi(target.theta - current_.theta);

        // Sphere metric: a step of d_theta covers cos(phi)·d_theta of arc, so
        // the budget in theta grows without bound as the pole is approached.
        // Capped at pi because beyond that "unbounded" and "any angle at all"
        // are the same thing.
        constexpr double kPoleCos = 1e-3;
        const double theta_budget = std::min(
            max_theta_rate_ * dt / std::max(std::cos(current_.phi), kPoleCos),
            M_PI);

        const double phi_budget = max_phi_rate_ * dt;

        current_.theta = wrap_pi(
            current_.theta + clamp(d_theta, -theta_budget, theta_budget));
        current_.phi = clamp(
            current_.phi + clamp(target.phi - current_.phi, -phi_budget, phi_budget),
            -M_PI / 2.0, M_PI / 2.0);
        current_.m = target.m;

        return current_;
    }

    /**
     * Jump straight to `target`, ignoring the rate limits.
     *
     * Only legitimate while the drives are at zero. Rate-limiting the shape is
     * what stops the wheels fighting the ground mid-manoeuvre; with no torque
     * being delivered there is nothing to fight, and the steering joints are
     * still rate-limited by the controller's own step_angle(). Snapping lets a
     * standing rover pivot straight to where it is actually going instead of
     * tracking a sweep it is not travelling along.
     */
    TwistShape snap(TwistShape target)
    {
        target = resolve_antipode(target, current_);
        current_.theta = wrap_pi(target.theta);
        current_.phi   = clamp(target.phi, -M_PI / 2.0, M_PI / 2.0);
        current_.m     = target.m;
        return current_;
    }

    const TwistShape & current() const { return current_; }

    void reset()
    {
        current_ = TwistShape{};
    }

private:
    double     max_theta_rate_;
    double     max_phi_rate_;
    TwistShape current_{};
};

}  // namespace rover_controller
