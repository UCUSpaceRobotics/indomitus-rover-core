// ─────────────────────────────────────────────────────────────────────────────
// test_swerve_controller_test.cpp
//
// Tests for RoverSwerveControllerTest, the experimental shape/magnitude swerve
// controller. The name is unfortunate but honest: it is the test suite for the
// controller whose class name ends in "Test".
//
// The fixture drives the controller the way controller_manager does — assign
// loaned interfaces, activate, call update() in a loop — with the plant faked
// by writing state interfaces from command interfaces. That fake is where the
// interesting knobs are: perfect_tracking() is a rover that follows exactly,
// and the individual freeze/stall helpers are the failure modes the review
// asked about (lagging steering feedback, wheels still rolling after the
// command goes to zero).
// ─────────────────────────────────────────────────────────────────────────────

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_controller/swerve_controller_test.hpp"

// ControllerInterfaceBase::init() was reworked between the distros this package
// builds on: Humble takes (name, namespace, node_options); Jazzy and later take
// a ControllerInterfaceParams struct and deprecate the positional form. Detect
// by header rather than by ROS_DISTRO, so this keeps working through the next
// rename too.
#if defined(__has_include)
#  if __has_include("controller_interface/controller_interface_params.hpp")
#    include "controller_interface/controller_interface_params.hpp"
#    define ROVER_CONTROLLER_INIT_TAKES_PARAMS 1
#  endif
#endif

using rover_controller::NUM_WHEELS;
using rover_controller::RoverSwerveControllerTest;

namespace {

constexpr const char * kControllerName = "swerve_controller_test";

const std::array<std::string, NUM_WHEELS> kSteerJoints = {
    "fl_wheel_mount_joint", "fr_wheel_mount_joint",
    "bl_wheel_mount_joint", "br_wheel_mount_joint"};
const std::array<std::string, NUM_WHEELS> kDriveJoints = {
    "fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint"};

// Geometry and limits used by every test unless overridden. Deliberately the
// shape of the real rover rather than a unit square, so an accidental
// wheelbase/track transposition shows up.
constexpr double kWheelbase     = 0.80;
constexpr double kTrackWidth    = 0.60;
constexpr double kWheelRadius   = 0.15;
constexpr double kMaxLinear     = 1.0;
constexpr double kMaxSteerDeg   = 135.0;
constexpr double kSteerRateDeg  = 120.0;
constexpr double kParkSpeed     = 0.001;
constexpr double kStandstill    = 0.02;
constexpr double kStandstillHold = 0.2;

constexpr double       kDt         = 0.01;   // matching the real control loop
constexpr unsigned int kUpdateRate = 100;    // Hz — the same figure, as init() wants it

/// init() across distros. See the header note above.
controller_interface::return_type init_controller(
    RoverSwerveControllerTest & controller,
    const std::string & name,
    const rclcpp::NodeOptions & options)
{
#ifdef ROVER_CONTROLLER_INIT_TAKES_PARAMS
    controller_interface::ControllerInterfaceParams params;
    params.controller_name                = name;
    params.node_options                   = options;
    params.update_rate                    = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    return controller.init(params);
#else
    return controller.init(name, "", options);
#endif
}

std::vector<rclcpp::Parameter> default_parameters()
{
    return {
        rclcpp::Parameter("steer_joint_names",
            std::vector<std::string>(kSteerJoints.begin(), kSteerJoints.end())),
        rclcpp::Parameter("drive_joint_names",
            std::vector<std::string>(kDriveJoints.begin(), kDriveJoints.end())),
        rclcpp::Parameter("wheelbase", kWheelbase),
        rclcpp::Parameter("track_width", kTrackWidth),
        rclcpp::Parameter("wheel_radius", kWheelRadius),
        rclcpp::Parameter("max_steer_deg", kMaxSteerDeg),
        rclcpp::Parameter("max_steer_rate_deg", kSteerRateDeg),
        rclcpp::Parameter("max_linear_speed", kMaxLinear),
        rclcpp::Parameter("max_accel", 2.0),
        rclcpp::Parameter("max_decel", 4.0),
        // Effectively disabled by default. The fixture's clock is a fake that
        // advances kDt per update() while /cmd_vel is stamped from the node's
        // real clock, so the two only stay comparable over a short run. Tests
        // step whole simulated seconds; the timeout gets its own test, which
        // syncs the two clocks and keeps the run short.
        rclcpp::Parameter("cmd_vel_timeout_s", 1.0e6),
        rclcpp::Parameter("rotation_scale_length", 0.0),
        rclcpp::Parameter("max_theta_rate_rad", 1.5708),
        rclcpp::Parameter("max_phi_rate_rad", 1.0),
        rclcpp::Parameter("park_speed", kParkSpeed),
        rclcpp::Parameter("standstill_speed", kStandstill),
        rclcpp::Parameter("standstill_hold_s", kStandstillHold),
        rclcpp::Parameter("idle_home_delay", 0.0),
    };
}

}  // namespace


class SwerveControllerTestFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        controller_ = std::make_shared<RoverSwerveControllerTest>();
    }

    void TearDown() override
    {
        stop_executor();
        controller_.reset();
    }

    // ── Bring-up ────────────────────────────────────────────────────────────

    /// init() + on_configure(). `overrides` are applied on top of the defaults;
    /// later entries win, which is how the invalid-parameter tests work.
    controller_interface::CallbackReturn configure(
        const std::vector<rclcpp::Parameter> & overrides = {})
    {
        auto params = default_parameters();
        params.insert(params.end(), overrides.begin(), overrides.end());

        const auto options = rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
            .parameter_overrides(params);

        if (init_controller(*controller_, kControllerName, options) !=
            controller_interface::return_type::OK)
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_->on_configure(rclcpp_lifecycle::State());
    }

    controller_interface::CallbackReturn activate()
    {
        assign_interfaces();
        const auto result = controller_->on_activate(rclcpp_lifecycle::State());
        // Start the fake clock where the controller's own clock is, so the
        // /cmd_vel stamps written by the subscription callback are comparable
        // with the times handed to update().
        now_ = controller_->get_node()->get_clock()->now();
        return result;
    }

    void configure_and_activate(const std::vector<rclcpp::Parameter> & overrides = {})
    {
        ASSERT_EQ(configure(overrides), controller_interface::CallbackReturn::SUCCESS);
        ASSERT_EQ(activate(), controller_interface::CallbackReturn::SUCCESS);
    }

    void assign_interfaces()
    {
        // The loaned wrappers hold references, so the underlying interface
        // objects must not move afterwards — reserve up front.
        command_storage_.reserve(NUM_WHEELS * 2);
        state_storage_.reserve(NUM_WHEELS * 2);

        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            command_storage_.emplace_back(
                kSteerJoints[i], hardware_interface::HW_IF_POSITION, &steer_cmd_[i]);
            state_storage_.emplace_back(
                kSteerJoints[i], hardware_interface::HW_IF_POSITION, &steer_pos_[i]);
        }
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            command_storage_.emplace_back(
                kDriveJoints[i], hardware_interface::HW_IF_VELOCITY, &drive_cmd_[i]);
            state_storage_.emplace_back(
                kDriveJoints[i], hardware_interface::HW_IF_VELOCITY, &drive_vel_[i]);
        }

        std::vector<hardware_interface::LoanedCommandInterface> commands;
        std::vector<hardware_interface::LoanedStateInterface>   states;
        for (auto & iface : command_storage_) { commands.emplace_back(iface); }
        for (auto & iface : state_storage_)   { states.emplace_back(iface); }

        controller_->assign_interfaces(std::move(commands), std::move(states));
    }

    // ── The fake plant ──────────────────────────────────────────────────────
    //
    // Public because test bodies take member pointers to them to pass to
    // step(); a TEST_F body is a derived class, which cannot form a pointer to
    // a protected member of the fixture.
public:
    /// A rover that follows its commands exactly. The default between cycles.
    void perfect_tracking()
    {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            steer_pos_[i] = steer_cmd_[i];
            drive_vel_[i] = drive_cmd_[i];
        }
    }

    /// Steering feedback that never arrives — joints report where they started
    /// no matter what is commanded.
    void frozen_steering_feedback()
    {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            drive_vel_[i] = drive_cmd_[i];
        }
    }

    /// Drives still turning at `rate` rad/s regardless of command — the rover
    /// coasting, or a wheel the controller cannot actually stop this instant.
    void coasting(double rate)
    {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            steer_pos_[i] = steer_cmd_[i];
            drive_vel_[i] = rate;
        }
    }

protected:
    // ── Driving the loop ────────────────────────────────────────────────────

    /// Publish a command straight into the controller's buffer via its topic.
    /// Goes through the real subscription so validation and the RT handoff are
    /// exercised, not bypassed.
    void send_cmd_vel(double vx, double vy, double wz)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x  = vx;
        msg.linear.y  = vy;
        msg.angular.z = wz;
        publish_and_settle(msg);
    }

    void publish_and_settle(const geometry_msgs::msg::Twist & msg)
    {
        ensure_helper_node();
        cmd_vel_pub_->publish(msg);
        // Give the controller's executor a moment to deliver it. The controller
        // node is spun by run_executor(); tests that do not need concurrency
        // spin it inline here instead.
        for (int i = 0; i < 50 && !executor_running_; ++i) {
            rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (executor_running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }

    /// One control cycle. Advances the fake clock by kDt, ticks the plant, then
    /// calls update().
    controller_interface::return_type step(void (SwerveControllerTestFixture::*plant)() =
                                               &SwerveControllerTestFixture::perfect_tracking)
    {
        (this->*plant)();
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        return controller_->update(now_, rclcpp::Duration::from_seconds(kDt));
    }

    void step_for(double seconds,
                  void (SwerveControllerTestFixture::*plant)() =
                      &SwerveControllerTestFixture::perfect_tracking)
    {
        const int cycles = static_cast<int>(std::round(seconds / kDt));
        for (int i = 0; i < cycles; ++i) {
            ASSERT_EQ(step(plant), controller_interface::return_type::OK);
        }
    }

    // ── Helper node / executor ──────────────────────────────────────────────

    void ensure_helper_node()
    {
        if (helper_node_) { return; }
        helper_node_ = std::make_shared<rclcpp::Node>("swerve_test_helper");
        cmd_vel_pub_ = helper_node_->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SystemDefaultsQoS());
        compact_client_ = helper_node_->create_client<std_srvs::srv::SetBool>(
            std::string("/") + kControllerName + "/set_compact_mode");
    }

    /// Spin the controller node on its own thread, so subscription and service
    /// callbacks land concurrently with update() exactly as they do at runtime.
    void run_executor()
    {
        ensure_helper_node();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(controller_->get_node()->get_node_base_interface());
        executor_->add_node(helper_node_);
        executor_running_ = true;
        executor_thread_ = std::thread([this]() { executor_->spin(); });
    }

    void stop_executor()
    {
        if (!executor_running_) { return; }
        executor_->cancel();
        if (executor_thread_.joinable()) { executor_thread_.join(); }
        executor_running_ = false;
    }

    // ── Assertions / readouts ───────────────────────────────────────────────

    /// Reconstruct wheel i's commanded ground-velocity direction.
    ///
    /// (angle, speed) and (angle ± pi, -speed) describe the same physical
    /// motion, and the controller is free to pick either — so tests must not
    /// assert on the angle directly. This product is invariant under that
    /// choice, which makes it the thing worth asserting on.
    std::pair<double, double> wheel_velocity(std::size_t i) const
    {
        return {drive_cmd_[i] * std::cos(steer_cmd_[i]),
                drive_cmd_[i] * std::sin(steer_cmd_[i])};
    }

    double max_abs_drive_cmd() const
    {
        double worst = 0.0;
        for (const double v : drive_cmd_) { worst = std::max(worst, std::abs(v)); }
        return worst;
    }

    std::array<double, NUM_WHEELS> steer_commands() const { return steer_cmd_; }

    void expect_all_finite() const
    {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            EXPECT_TRUE(std::isfinite(steer_cmd_[i])) << "steer " << i;
            EXPECT_TRUE(std::isfinite(drive_cmd_[i])) << "drive " << i;
        }
    }

    // Plant storage. Command interfaces are written by the controller; state
    // interfaces are written by the fake plant.
    std::array<double, NUM_WHEELS> steer_cmd_{};
    std::array<double, NUM_WHEELS> steer_pos_{};
    std::array<double, NUM_WHEELS> drive_cmd_{};
    std::array<double, NUM_WHEELS> drive_vel_{};

    std::vector<hardware_interface::CommandInterface> command_storage_;
    std::vector<hardware_interface::StateInterface>   state_storage_;

    std::shared_ptr<RoverSwerveControllerTest> controller_;

    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       compact_client_;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    bool        executor_running_{false};

    rclcpp::Time now_{0, 0, RCL_ROS_TIME};
};


// ─────────────────────────────────────────────────────────────────────────────
// Parameter validation
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, ConfiguresWithDefaults)
{
    EXPECT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveControllerTestFixture, RejectsNonPositiveMaxLinearSpeed)
{
    EXPECT_EQ(configure({rclcpp::Parameter("max_linear_speed", -1.0)}),
              controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTestFixture, RejectsNonFiniteGeometry)
{
    EXPECT_EQ(
        configure({rclcpp::Parameter(
            "wheelbase", std::numeric_limits<double>::quiet_NaN())}),
        controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTestFixture, RejectsInfiniteRateLimit)
{
    EXPECT_EQ(
        configure({rclcpp::Parameter(
            "max_phi_rate_rad", std::numeric_limits<double>::infinity())}),
        controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTestFixture, RejectsZeroWheelRadius)
{
    EXPECT_EQ(configure({rclcpp::Parameter("wheel_radius", 0.0)}),
              controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTestFixture, AcceptsNegativeIdleHomeDelayAsDisabled)
{
    // Negative is the documented "never home" setting, not a mistake.
    EXPECT_EQ(configure({rclcpp::Parameter("idle_home_delay", -1.0)}),
              controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveControllerTestFixture, ClaimsDriveVelocityStateInterfaces)
{
    ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);

    const auto states = controller_->state_interface_configuration().names;
    for (const auto & joint : kDriveJoints) {
        const std::string wanted = joint + "/" + hardware_interface::HW_IF_VELOCITY;
        EXPECT_NE(std::find(states.begin(), states.end(), wanted), states.end())
            << "standstill detection needs " << wanted;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Non-finite input
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, NonFiniteTwistIsRejectedBeforeKinematics)
{
    configure_and_activate();

    send_cmd_vel(0.5, 0.0, 0.3);
    step_for(0.5);

    const auto before = steer_commands();

    geometry_msgs::msg::Twist bad;
    bad.linear.x  = std::numeric_limits<double>::quiet_NaN();
    bad.linear.y  = 0.0;
    bad.angular.z = std::numeric_limits<double>::infinity();
    publish_and_settle(bad);

    step_for(0.2);

    expect_all_finite();
    // Rejected, not adopted: the last good command is still in force, so the
    // steering has not moved.
    const auto after = steer_commands();
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        EXPECT_NEAR(before[i], after[i], 1e-9) << "wheel " << i;
    }
}

TEST_F(SwerveControllerTestFixture, NonFiniteSteeringFeedbackHoldsDrivesAtZero)
{
    configure_and_activate();

    send_cmd_vel(0.5, 0.0, 0.0);
    step_for(0.5);
    ASSERT_GT(max_abs_drive_cmd(), 0.1) << "precondition: rover should be driving";

    steer_pos_[1] = std::numeric_limits<double>::quiet_NaN();
    // Plant that keeps re-poisoning the reading rather than letting the next
    // perfect_tracking() call wash it away.
    for (int i = 0; i < 20; ++i) {
        steer_pos_[1] = std::numeric_limits<double>::quiet_NaN();
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        ASSERT_EQ(controller_->update(now_, rclcpp::Duration::from_seconds(kDt)),
                  controller_interface::return_type::OK);
    }

    expect_all_finite();
    EXPECT_DOUBLE_EQ(max_abs_drive_cmd(), 0.0)
        << "no trustworthy steering feedback means no idea which way to drive";
}

TEST_F(SwerveControllerTestFixture, NonFiniteDriveFeedbackNeverDeclaresStandstill)
{
    configure_and_activate();

    send_cmd_vel(0.5, 0.0, 0.6);
    step_for(0.6);
    const auto driving = steer_commands();

    send_cmd_vel(0.0, 0.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            steer_pos_[w] = steer_cmd_[w];
            drive_vel_[w] = std::numeric_limits<double>::quiet_NaN();
        }
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        ASSERT_EQ(controller_->update(now_, rclcpp::Duration::from_seconds(kDt)),
                  controller_interface::return_type::OK);
    }

    // Without evidence of standstill the wheels must hold their angle rather
    // than home. Homing would have driven every command to ~0.
    const auto after = steer_commands();
    double moved = 0.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        moved = std::max(moved, std::abs(after[i] - driving[i]));
    }
    EXPECT_LT(moved, 0.05) << "homed on unusable drive feedback";
}


// ─────────────────────────────────────────────────────────────────────────────
// Standstill: no snap, no homing while still rolling
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, CoastingDoesNotHomeTheWheels)
{
    // idle_home_delay 0.0 is the setting the review flagged: with the old
    // limiter-based test, zero cmd_vel homed the wheels the instant the
    // *commanded* speed reached zero, which is the start of stopping.
    configure_and_activate();

    send_cmd_vel(0.6, 0.3, 0.0);
    step_for(1.0);

    const auto driving = steer_commands();
    ASSERT_GT(std::abs(driving[0]), 0.1) << "precondition: wheels should be crabbed";

    // Command stops; wheels keep turning well above the standstill threshold.
    send_cmd_vel(0.0, 0.0, 0.0);
    const double rolling_rate = 4.0 * kStandstill / kWheelRadius;
    for (int i = 0; i < 200; ++i) {
        coasting(rolling_rate);
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        ASSERT_EQ(controller_->update(now_, rclcpp::Duration::from_seconds(kDt)),
                  controller_interface::return_type::OK);

        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            EXPECT_NEAR(steer_cmd_[w], driving[w], 1e-6)
                << "wheel " << w << " pivoted at cycle " << i << " while still rolling";
        }
    }
}

TEST_F(SwerveControllerTestFixture, HomesOnlyAfterMeasuredStandstillHolds)
{
    configure_and_activate();

    send_cmd_vel(0.6, 0.3, 0.0);
    step_for(1.0);
    const auto driving = steer_commands();
    ASSERT_GT(std::abs(driving[0]), 0.1);

    send_cmd_vel(0.0, 0.0, 0.0);

    // Wheels stop, but the hold window has not elapsed yet.
    step_for(kStandstillHold * 0.5);
    for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
        EXPECT_NEAR(steer_cmd_[w], driving[w], 1e-6)
            << "wheel " << w << " homed before the standstill hold elapsed";
    }

    // Now give it long enough to confirm standstill and walk home.
    step_for(3.0);
    for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
        EXPECT_NEAR(steer_cmd_[w], 0.0, 0.02) << "wheel " << w << " failed to home";
    }
}

TEST_F(SwerveControllerTestFixture, NoSnapWhileRollingEvenOnALargeShapeChange)
{
    configure_and_activate();

    send_cmd_vel(0.8, 0.0, 0.0);
    step_for(1.0);

    // Ask for a pure spin — the largest shape change there is — while the
    // wheels are reported as still turning. snap() must not fire, so every
    // steering command must stay inside the joint rate limit.
    send_cmd_vel(0.0, 0.0, 1.0);

    const double rate_limit = kSteerRateDeg * M_PI / 180.0;
    const double rolling_rate = 4.0 * kStandstill / kWheelRadius;
    auto previous = steer_commands();

    for (int i = 0; i < 200; ++i) {
        coasting(rolling_rate);
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        ASSERT_EQ(controller_->update(now_, rclcpp::Duration::from_seconds(kDt)),
                  controller_interface::return_type::OK);

        const auto current = steer_commands();
        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            EXPECT_LE(std::abs(current[w] - previous[w]), rate_limit * kDt + 1e-9)
                << "wheel " << w << " jumped at cycle " << i;
        }
        previous = current;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Shape / magnitude separation
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, ThrottleOnlyChangeLeavesSteeringUntouched)
{
    // The reason this controller exists. Halving the twist while keeping its
    // ratios changes m alone; theta and phi — and so every steering angle —
    // must not move at all.
    configure_and_activate();

    send_cmd_vel(0.8, 0.2, 0.5);
    step_for(2.0);
    const auto settled = steer_commands();

    send_cmd_vel(0.4, 0.1, 0.25);
    step_for(1.0);
    const auto after = steer_commands();

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        EXPECT_NEAR(settled[i], after[i], 1e-6)
            << "wheel " << i << " moved on a pure throttle change";
    }
    EXPECT_GT(max_abs_drive_cmd(), 0.0) << "still expected to be driving";
}

TEST_F(SwerveControllerTestFixture, PureSpinProducesTangentialWheelVelocities)
{
    configure_and_activate();

    send_cmd_vel(0.0, 0.0, 0.8);
    step_for(3.0);

    ASSERT_GT(max_abs_drive_cmd(), 0.1) << "spin should be driving the wheels";

    // For a spin about the centre, wheel i's ground velocity is wz x r_i.
    // Compare directions rather than angles, so the ±180° representation
    // choice does not matter.
    const double L = kWheelbase / 2.0;
    const double W = kTrackWidth / 2.0;
    const std::array<std::pair<double, double>, NUM_WHEELS> expected = {{
        {-W, +L},   // FL
        {+W, +L},   // FR
        {-W, -L},   // RL
        {+W, -L},   // RR
    }};

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto [vx, vy] = wheel_velocity(i);
        const double norm = std::hypot(vx, vy);
        ASSERT_GT(norm, 1e-6) << "wheel " << i << " not commanded";

        const auto [ex, ey] = expected[i];
        const double enorm = std::hypot(ex, ey);
        const double cos_between = (vx * ex + vy * ey) / (norm * enorm);
        EXPECT_NEAR(cos_between, 1.0, 1e-3)
            << "wheel " << i << " points the wrong way for a spin";
    }
}

TEST_F(SwerveControllerTestFixture, TransitionIntoAndOutOfSpinStaysContinuous)
{
    configure_and_activate();

    const double rate_limit = kSteerRateDeg * M_PI / 180.0;

    auto drive_and_check = [&](double vx, double vy, double wz, double seconds) {
        send_cmd_vel(vx, vy, wz);
        auto previous = steer_commands();
        const int cycles = static_cast<int>(std::round(seconds / kDt));
        for (int i = 0; i < cycles; ++i) {
            ASSERT_EQ(step(), controller_interface::return_type::OK);
            const auto current = steer_commands();
            for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
                EXPECT_LE(std::abs(current[w] - previous[w]), rate_limit * kDt + 1e-9)
                    << "wheel " << w << " jumped during transition";
            }
            previous = current;
        }
    };

    drive_and_check(0.8, 0.0, 0.0, 1.0);    // straight
    drive_and_check(0.0, 0.0, 1.0, 3.0);    // into a spin
    drive_and_check(0.8, 0.0, 0.0, 3.0);    // and back out

    expect_all_finite();
}

TEST_F(SwerveControllerTestFixture, ForwardToReverseKeepsSteeringSteady)
{
    // Reversing along the same arc is a sign flip on m, not a 180° sweep of
    // the wheels: the antipodal shape has identical steering angles.
    configure_and_activate();

    send_cmd_vel(0.6, 0.0, 0.4);
    step_for(2.0);
    const auto forward = steer_commands();

    send_cmd_vel(-0.6, 0.0, -0.4);
    step_for(2.0);
    const auto reverse = steer_commands();

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        EXPECT_NEAR(forward[i], reverse[i], 0.02)
            << "wheel " << i << " swung around on a direction change";
    }

    // And it really is reversing: every wheel's ground velocity flipped.
    EXPECT_GT(max_abs_drive_cmd(), 0.05);
}

TEST_F(SwerveControllerTestFixture, NearZeroTwistSteersWithoutDriving)
{
    // The "point the wheels here" command: a twist far below park_speed still
    // carries a direction, and normalising recovers it at any scale.
    configure_and_activate();

    const double probe = kParkSpeed * 0.01;
    send_cmd_vel(probe, 0.0, probe * 2.0);
    step_for(3.0);

    EXPECT_DOUBLE_EQ(max_abs_drive_cmd(), 0.0) << "drives must stay parked";

    double steered = 0.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steered = std::max(steered, std::abs(steer_cmd_[i]));
    }
    EXPECT_GT(steered, 0.05) << "wheels should have taken up the commanded angle";
}


// ─────────────────────────────────────────────────────────────────────────────
// Steering feedback lag
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, LaggingSteeringFeedbackCutsDrive)
{
    // A hard crab: the wheels must swing ~90°. With feedback that never
    // arrives, the settled state is a 90° measured error, where cos⁴ is zero
    // and the drives must be held down — however long the controller waits.
    configure_and_activate();

    send_cmd_vel(0.0, 0.8, 0.0);
    step_for(3.0, &SwerveControllerTestFixture::frozen_steering_feedback);

    EXPECT_LT(max_abs_drive_cmd(), 0.01)
        << "drove with the wheels reported 90° off-target";

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        EXPECT_NEAR(std::abs(steer_cmd_[i]), M_PI / 2.0, 0.05)
            << "wheel " << i << " should still be commanded to the crab angle";
    }
}

TEST_F(SwerveControllerTestFixture, SameCommandDrivesNormallyWhenFeedbackTracks)
{
    // The control for the test above: identical command, working feedback.
    // Without this, "drives are zero" could just mean the command was wrong.
    configure_and_activate();

    send_cmd_vel(0.0, 0.8, 0.0);
    step_for(3.0);

    EXPECT_GT(max_abs_drive_cmd(), 0.8 * 0.8 / kWheelRadius)
        << "should be driving at close to the commanded speed";
}

TEST_F(SwerveControllerTestFixture, DriveCommandStaysContinuousThroughTheDirectionFlip)
{
    // With the scale taken from feedback and the sign from the integrated
    // command, the sign flipped where the scale was still large and the drive
    // command jumped. One shared angle source ties the flip to the zero.
    configure_and_activate();

    send_cmd_vel(0.7, 0.0, 0.0);
    step_for(1.0);

    // Reverse the crab direction so each wheel's target crosses 90° away from
    // where it currently is, with the feedback lagging behind the command.
    send_cmd_vel(-0.05, 0.7, 0.0);

    std::array<double, NUM_WHEELS> previous = drive_cmd_;
    for (int i = 0; i < 300; ++i) {
        // Feedback that trails the command by a fixed fraction of the step.
        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            steer_pos_[w] += 0.5 * (steer_cmd_[w] - steer_pos_[w]);
            drive_vel_[w] = drive_cmd_[w];
        }
        now_ = now_ + rclcpp::Duration::from_seconds(kDt);
        ASSERT_EQ(controller_->update(now_, rclcpp::Duration::from_seconds(kDt)),
                  controller_interface::return_type::OK);

        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            EXPECT_LT(std::abs(drive_cmd_[w] - previous[w]), 0.5)
                << "wheel " << w << " drive command jumped at cycle " << i;
            previous[w] = drive_cmd_[w];
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// cmd_vel timeout
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, StaleCommandStopsTheDrives)
{
    // Short run so the fake clock and the node clock (which stamps /cmd_vel)
    // stay close enough to compare — see the note on cmd_vel_timeout_s above.
    configure_and_activate({rclcpp::Parameter("cmd_vel_timeout_s", 0.5)});

    send_cmd_vel(0.8, 0.0, 0.0);
    step_for(0.2);
    ASSERT_GT(max_abs_drive_cmd(), 0.1) << "should be driving before the timeout";

    step_for(1.0);   // nothing more published
    EXPECT_DOUBLE_EQ(max_abs_drive_cmd(), 0.0);
}


// ─────────────────────────────────────────────────────────────────────────────
// Concurrency — the real handoffs, on real threads
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SwerveControllerTestFixture, ConcurrentCmdVelUpdatesStayWithinLimits)
{
    // The whole point of the RealtimeBuffer. A publisher thread alternates
    // between two twists whose shapes are far apart; update() runs flat out on
    // this thread. Every cycle must still be a legal, rate-limited step —
    // a torn read pairing one twist's vx with the other's wz would show up as
    // a shape, and so a steering command, that neither twist asked for.
    configure_and_activate();
    run_executor();

    std::atomic<bool> publishing{true};
    std::thread writer([this, &publishing]() {
        bool flip = false;
        while (publishing) {
            geometry_msgs::msg::Twist msg;
            if (flip) {
                msg.linear.x = 0.9;  msg.linear.y = 0.0;  msg.angular.z = 0.0;
            } else {
                msg.linear.x = 0.0;  msg.linear.y = -0.9; msg.angular.z = 1.4;
            }
            flip = !flip;
            cmd_vel_pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    });

    const double rate_limit  = kSteerRateDeg * M_PI / 180.0;
    const double steer_limit = kMaxSteerDeg * M_PI / 180.0;
    const double speed_limit = kMaxLinear / kWheelRadius;

    auto previous = steer_commands();
    for (int i = 0; i < 3000; ++i) {
        ASSERT_EQ(step(), controller_interface::return_type::OK);
        const auto current = steer_commands();
        for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
            ASSERT_TRUE(std::isfinite(current[w])) << "wheel " << w << " cycle " << i;
            ASSERT_TRUE(std::isfinite(drive_cmd_[w])) << "wheel " << w << " cycle " << i;
            EXPECT_LE(std::abs(current[w] - previous[w]), rate_limit * kDt + 1e-9)
                << "wheel " << w << " jumped at cycle " << i;
            EXPECT_LE(std::abs(current[w]), steer_limit + 1e-6);
            EXPECT_LE(std::abs(drive_cmd_[w]), speed_limit + 1e-6);
        }
        previous = current;
    }

    publishing = false;
    writer.join();
}

TEST_F(SwerveControllerTestFixture, CompactModeChangeWhileActiveIsRateLimited)
{
    configure_and_activate();
    run_executor();

    send_cmd_vel(0.5, 0.0, 0.0);
    step_for(1.0);

    ASSERT_TRUE(compact_client_->wait_for_service(std::chrono::seconds(5)));

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = compact_client_->async_send_request(request);

    // Keep the control loop running *while the service call is in flight* —
    // that is the race the review asked about. The kinematics object is only
    // ever mutated by update(), so every cycle stays a legal step.
    const double rate_limit = kSteerRateDeg * M_PI / 180.0;
    auto previous = steer_commands();

    auto step_checking_continuity = [&](int cycles) {
        for (int i = 0; i < cycles; ++i) {
            ASSERT_EQ(step(), controller_interface::return_type::OK);
            const auto current = steer_commands();
            for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
                ASSERT_TRUE(std::isfinite(current[w]));
                EXPECT_LE(std::abs(current[w] - previous[w]), rate_limit * kDt + 1e-9)
                    << "wheel " << w << " jumped at cycle " << i;
            }
            previous = current;
        }
    };

    // Drive the loop until the call comes back, so the two really do overlap.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "service never answered";
        step_checking_continuity(10);
    }
    EXPECT_TRUE(future.get()->success);

    // Then give the joints time to walk the ±pi fold at max_steer_rate.
    step_checking_continuity(static_cast<int>(std::round(4.0 / kDt)));

    // Compact mode folds the wheels by ±pi, so they end up far from where a
    // plain forward command would put them.
    double folded = 0.0;
    for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
        folded = std::max(folded, std::abs(steer_cmd_[w]));
    }
    EXPECT_GT(folded, 1.0) << "compact mode never took effect";
}

TEST_F(SwerveControllerTestFixture, CompactModeRequestBeforeActivationIsHonoured)
{
    ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
    ensure_helper_node();
    run_executor();

    ASSERT_TRUE(compact_client_->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = compact_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_TRUE(future.get()->success);

    ASSERT_EQ(activate(), controller_interface::CallbackReturn::SUCCESS);
    step_for(3.0);

    double folded = 0.0;
    for (std::size_t w = 0; w < NUM_WHEELS; ++w) {
        folded = std::max(folded, std::abs(steer_cmd_[w]));
    }
    EXPECT_GT(folded, 1.0) << "request made while inactive was dropped";
}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
