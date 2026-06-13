#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace rover_chassis_controller {

constexpr double VXY_EPS = 1e-3;   ///< m/s  — below this vx/vy is considered zero
constexpr double WZ_EPS  = 1e-3;   ///< rad/s — below this wz is considered zero

// ─────────────────────────────────────────────────────────────────────────────
// WheelData — strongly-typed container for per-wheel scalars
//
// Index layout (used for array access throughout):
//   0 = FL (front-left)
//   1 = FR (front-right)
//   2 = RL (rear-left)
//   3 = RR (rear-right)
// ─────────────────────────────────────────────────────────────────────────────

struct WheelData {
    std::array<double, 4> data{};

    double & fl() { return data[0]; }
    double & fr() { return data[1]; }
    double & rl() { return data[2]; }
    double & rr() { return data[3]; }

    double fl() const { return data[0]; }
    double fr() const { return data[1]; }
    double rl() const { return data[2]; }
    double rr() const { return data[3]; }

    double &       operator[](std::size_t i)       { return data[i]; }
    const double & operator[](std::size_t i) const { return data[i]; }

    static WheelData filled(double v)
    {
        WheelData w;
        w.data.fill(v);
        return w;
    }

    /// Compact-mode steering offsets: wheels rotate ±π to fold inward
    static WheelData compact_offsets()
    {
        WheelData w;
        w.fl() = -M_PI;
        w.fr() = +M_PI;
        w.rl() = +M_PI;
        w.rr() = -M_PI;
        return w;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// IK result — angles + wheel speeds packed together
// ─────────────────────────────────────────────────────────────────────────────

struct IKResult {
    WheelData angles;
    WheelData speeds;
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}


class SwerveKinematics {
public:
    // ── Construction ───────────────────────────────────────────────────────────

    /**
     * @param wheelbase    Distance between front and rear axles [m]
     * @param track_width  Distance between left and right wheels [m]
     * @param wheel_radius Driven wheel radius [m]
     * @param max_steer    Hard steering joint limit [rad], symmetric ±
     * @param max_linear   Maximum linear wheel speed [m/s]
     */
    SwerveKinematics(
        double wheelbase,
        double track_width,
        double wheel_radius,
        double max_steer,
        double max_linear)
    : wheel_radius_(wheel_radius),
        max_steer_(max_steer),
        max_linear_(max_linear),
        half_wheelbase_(wheelbase / 2.0),
        half_track_(track_width / 2.0),
        compact_mode_(false),
        offset_angles_(WheelData::filled(0.0))
    {}

    // ── Compact mode ───────────────────────────────────────────────────────────

    void set_compact_mode(bool enabled)
    {
        compact_mode_ = enabled;
        offset_angles_ = enabled ? WheelData::compact_offsets() : WheelData::filled(0.0);
    }

    bool compact_mode() const { return compact_mode_; }

    WheelData offset_angles() const { return offset_angles_; }

    // ── Public IK entry points ─────────────────────────────────────────────────

    /**
     * Full holonomic IK — supports translation, rotation, and combined motion.
     * Handles internal ICR points (e.g. tight turns where ICR is between wheels).
     *
     * @param vx               Chassis forward velocity [m/s]
     * @param vy               Chassis lateral velocity [m/s]
     * @param wz               Chassis yaw rate [rad/s]
     * @param current_angles   Current physical steering joint angles [rad]
     * @return                 Target angles [rad] and wheel speeds [rad/s]
     */
    IKResult ik_full(
        double vx, double vy, double wz,
        const WheelData & current_angles) const
    {
        if (std::abs(vx) < VXY_EPS && std::abs(vy) < VXY_EPS && std::abs(wz) < WZ_EPS) {
        return {apply_compact_offset(WheelData::filled(0.0)), WheelData::filled(0.0)};
        }

        auto [angles, speeds] = ik_general(vx, vy, wz, current_angles);
        angles = apply_compact_offset(angles);

        if (compact_mode_) {
        for (auto & s : speeds.data) { s = -s; }
        }

        return {angles, speeds};
    }

    /**
     * Pure rotate-in-place IK — ignores vx/vy, sets up symmetric diamond pattern.
     *
     * @param wz               Chassis yaw rate [rad/s]
     * @param current_angles   Current physical steering joint angles [rad]
     * @return                 Target angles [rad] and wheel speeds [rad/s]
     */
    IKResult ik_rotate(double wz, const WheelData & current_angles) const
    {
        auto [angles, speeds] = ik_general(0.0, 0.0, wz, current_angles);
        angles = apply_compact_offset(angles);

        if (compact_mode_) {
        for (auto & s : speeds.data) { s = -s; }
        }

        return {angles, speeds};
    }

private:
    // ── Core affine velocity mapping ───────────────────────────────────────────

    IKResult ik_general(
        double vx, double vy, double wz,
        const WheelData & current_angles) const
    {
        const double L = half_wheelbase_;
        const double W = half_track_;

        // 1. Per-wheel velocity vectors from rigid-body kinematics
        //    v_wheel = v_chassis + ω × r_wheel
        const double vel_x[4] = {
        vx - wz * W,   // FL
        vx + wz * W,   // FR
        vx - wz * W,   // RL
        vx + wz * W    // RR
        };
        const double vel_y[4] = {
        vy + wz * L,   // FL
        vy + wz * L,   // FR
        vy - wz * L,   // RL
        vy - wz * L    // RR
        };

        // 2. Raw wheel speeds [rad/s] from velocity magnitudes
        double speeds[4];
        for (int i = 0; i < 4; ++i) {
        speeds[i] = std::hypot(vel_x[i], vel_y[i]) / wheel_radius_;
        }

        // 3. Speed desaturation — scale all wheels proportionally if any exceeds limit
        const double max_allowable = max_linear_ / wheel_radius_;
        const double max_current   = *std::max_element(speeds, speeds + 4);
        if (max_current > max_allowable) {
        const double scale = max_allowable / max_current;
        for (auto & s : speeds) { s *= scale; }
        }

        // 4. Raw target angles from atan2
        double raw_angles[4];
        for (int i = 0; i < 4; ++i) {
        raw_angles[i] = std::atan2(vel_y[i], vel_x[i]);
        }

        // 5. Stateful angle optimisation — minimise travel, respect ±max_steer
        WheelData out_angles, out_speeds;
        for (int i = 0; i < 4; ++i) {
        auto [a, s] = optimize_wheel(raw_angles[i], speeds[i], current_angles[i]);
        out_angles[i] = a;
        out_speeds[i] = s;
        }

        return {out_angles, out_speeds};
    }

    // ── Per-wheel optimisation ─────────────────────────────────────────────────

    /**
     * Chooses between the direct angle and its 180° flip equivalent.
     * Prefers the option requiring least travel from current_angle.
     * Falls back to clamped angle if neither option fits within ±max_steer.
     *
     * @param target_angle   Desired wheel heading [rad], from atan2
     * @param target_speed   Desired wheel speed [rad/s]
     * @param current_angle  Current physical joint angle [rad]
     * @return               {optimised_angle, optimised_speed}
     */
    std::pair<double, double> optimize_wheel(
        double target_angle,
        double target_speed,
        double current_angle) const
    {
        const double opt1 = target_angle;
        const double opt2 = (target_angle < 0.0)
                            ? target_angle + M_PI
                            : target_angle - M_PI;

        const bool opt1_valid = (opt1 >= -max_steer_) && (opt1 <= max_steer_);
        const bool opt2_valid = (opt2 >= -max_steer_) && (opt2 <= max_steer_);

        if (opt1_valid && opt2_valid) {
        // Both valid — pick closest to current angle
        const double d1 = std::abs(opt1 - current_angle);
        const double d2 = std::abs(opt2 - current_angle);
        return (d1 <= d2)
                ? std::make_pair(opt1,  target_speed)
                : std::make_pair(opt2, -target_speed);
        }

        if (opt1_valid) { return {opt1,  target_speed}; }
        if (opt2_valid) { return {opt2, -target_speed}; }

        // Neither fits — clamp to closest limit of the nearer option
        const double d1 = std::abs(opt1 - current_angle);
        const double d2 = std::abs(opt2 - current_angle);
        return (d1 < d2)
            ? std::make_pair(clamp(opt1, -max_steer_, max_steer_),  target_speed)
            : std::make_pair(clamp(opt2, -max_steer_, max_steer_), -target_speed);
    }

    // Compact offset application

    WheelData apply_compact_offset(const WheelData & angles) const
    {
        if (!compact_mode_) { return angles; }

        WheelData out;
        for (std::size_t i = 0; i < 4; ++i) {
        out[i] = angles[i] + offset_angles_[i];
        }
        return out;
    }

    double wheel_radius_;
    double max_steer_;
    double max_linear_;
    double half_wheelbase_;
    double half_track_;

    bool     compact_mode_;
    WheelData offset_angles_;
};

}  // namespace rover_swerve_controller