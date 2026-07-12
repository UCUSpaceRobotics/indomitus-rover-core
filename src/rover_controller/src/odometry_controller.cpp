#include "rover_controller/odometry_controller.hpp"

#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

PLUGINLIB_EXPORT_CLASS(
    rover_controller::RoverOdometryController,
    controller_interface::ControllerInterface)

namespace rover_controller {

RoverOdometryController::RoverOdometryController()
: controller_interface::ControllerInterface()
{}


controller_interface::CallbackReturn
RoverOdometryController::on_init()
{
    try {
        declare_parameters();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[OdomController] on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverOdometryController::on_configure(const rclcpp_lifecycle::State & /*prev*/)
{
    if (!read_parameters()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    build_kinematics_matrix();

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/wheels", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(get_node()->get_logger(),
        "[OdomController] Configured — wheelbase=%.3f m  track=%.3f m  r_wheel=%.4f m",
        wheelbase_, track_width_, wheel_radius_);

    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverOdometryController::on_activate(const rclcpp_lifecycle::State & /*prev*/)
{
    if (!assign_interfaces()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    first_update_ = true;
    x_ = y_ = theta_ = 0.0;

    RCLCPP_INFO(get_node()->get_logger(), "[OdomController] Activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverOdometryController::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
{
    steer_handles_.reset();
    drive_handles_.reset();

    RCLCPP_INFO(get_node()->get_logger(), "[OdomController] Deactivated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
RoverOdometryController::command_interface_configuration() const
{
    // Pure observer — claims no command interfaces.
    return {controller_interface::interface_configuration_type::NONE, {}};
}


controller_interface::InterfaceConfiguration
RoverOdometryController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & name : steer_state_interface_names()) {
        cfg.names.push_back(name);
    }
    for (const auto & name : drive_state_interface_names()) {
        cfg.names.push_back(name);
    }
    return cfg;
}


controller_interface::return_type
RoverOdometryController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    if (!steer_handles_ || !drive_handles_) {
        return controller_interface::return_type::OK;
    }

    const double dt = period.seconds();
    if (dt <= 0.0) {
        return controller_interface::return_type::OK;
    }

    // 1. Read current encoder and steering values

    std::array<double, ODOM_NUM_WHEELS> steer_angles{};
    std::array<double, ODOM_NUM_WHEELS> drive_pos{};

    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
#if defined(JAZZY_OR_LATER)
        steer_angles[i] = steer_handles_->position[i].get().get_optional().value_or(0.0);
        drive_pos[i]    = drive_handles_->position[i].get().get_optional().value_or(0.0);
#else
        steer_angles[i] = steer_handles_->position[i].get().get_value();
        drive_pos[i]    = drive_handles_->position[i].get().get_value();
#endif
    }

    // Skip integration on the very first cycle — we only have one snapshot.
    if (first_update_) {
        for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
            prev_drive_pos_[i] = drive_pos[i];
        }
        first_update_ = false;

        // Publish the initial pose immediately (rover starts at rest at the
        // origin) so /odom and odom->base_link exist as soon as the controller
        // activates, before any motion occurs.
        publish_odom(0.0, 0.0, 0.0, time);
        return controller_interface::return_type::OK;
    }

    // 2. Compute per-wheel linear speed from encoder position delta

    Eigen::VectorXd b(8);   // [vx_0, vy_0, vx_1, vy_1, ...]

    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
        double delta_theta = drive_pos[i] - prev_drive_pos_[i];
        delta_theta -= 2.0 * M_PI * std::round(delta_theta / (2.0 * M_PI));
        const double speed       = wheel_radius_ * delta_theta / dt;
        const double angle       = steer_angles[i];

        b(2 * i    ) = speed * std::cos(angle);
        b(2 * i + 1) = speed * std::sin(angle);
    }

    prev_drive_pos_ = drive_pos;

    // 3. Least-squares estimate of chassis velocity

    const Eigen::Vector3d vel = A_pinv_ * b;   // [vx, vy, wz]

    double vx = vel(0);
    double vy = vel(1);
    double wz = vel(2);

    // Dead-zones — suppress noise when rover is stationary
    constexpr double VEL_EPS = 0.001;    // m/s
    constexpr double WZ_EPS  = 0.0001;   // rad/s

    if (std::abs(vx) < VEL_EPS) { vx = 0.0; }
    if (std::abs(vy) < VEL_EPS) { vy = 0.0; }
    if (std::abs(wz) < WZ_EPS)  { wz = 0.0; }

    // 4. Integrate pose using exact exponential map
    if (std::abs(wz) > 1e-9) {
        const double dx_local =
            ( vx * std::sin(wz * dt) - vy * (1.0 - std::cos(wz * dt))) / wz;
        const double dy_local =
            ( vx * (1.0 - std::cos(wz * dt)) + vy * std::sin(wz * dt)) / wz;

        x_ += dx_local * std::cos(theta_) - dy_local * std::sin(theta_);
        y_ += dx_local * std::sin(theta_) + dy_local * std::cos(theta_);
    } else {
        x_ += (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
        y_ += (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
    }

    theta_ += wz * dt;

    // 5. Publish

    publish_odom(vx, vy, wz, time);

    return controller_interface::return_type::OK;
}


void RoverOdometryController::declare_parameters()
{
    auto node = get_node();

    auto decl = [&](const std::string & name, auto default_val) {
        try { node->declare_parameter(name, default_val); }
        catch (const std::exception &) { /* already declared */ }
    };

    decl("wheelbase",    0.842);
    decl("track_width",  0.682);
    decl("wheel_radius", 0.16);

    decl("steer_joint_names",
        std::vector<std::string>{
            "fl_wheel_mount_joint", "fr_wheel_mount_joint",
            "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    decl("drive_joint_names",
        std::vector<std::string>{
            "fl_wheel_joint", "fr_wheel_joint",
            "bl_wheel_joint", "br_wheel_joint"});
}


bool RoverOdometryController::read_parameters()
{
    auto node = get_node();

    wheelbase_    = node->get_parameter("wheelbase").as_double();
    track_width_  = node->get_parameter("track_width").as_double();
    wheel_radius_ = node->get_parameter("wheel_radius").as_double();

    const auto steer_names = node->get_parameter("steer_joint_names").as_string_array();
    const auto drive_names = node->get_parameter("drive_joint_names").as_string_array();

    if (steer_names.size() != ODOM_NUM_WHEELS || drive_names.size() != ODOM_NUM_WHEELS) {
        RCLCPP_ERROR(node->get_logger(),
            "[OdomController] steer_joint_names and drive_joint_names "
            "must each have exactly %zu entries.", ODOM_NUM_WHEELS);
        return false;
    }

    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
        steer_joint_names_[i] = steer_names[i];
        drive_joint_names_[i] = drive_names[i];
    }

    return true;
}


void RoverOdometryController::build_kinematics_matrix()
{
    const double hx = wheelbase_  / 2.0;
    const double hy = track_width_ / 2.0;

    const std::array<std::pair<double, double>, ODOM_NUM_WHEELS> positions = {{
        { hx,  hy},   // FL
        { hx, -hy},   // FR
        {-hx,  hy},   // RL
        {-hx, -hy},   // RR
    }};

    Eigen::MatrixXd A(8, 3);
    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
        const double px = positions[i].first;
        const double py = positions[i].second;

        A(2 * i,     0) =  1.0;
        A(2 * i,     1) =  0.0;
        A(2 * i,     2) = -py;

        A(2 * i + 1, 0) =  0.0;
        A(2 * i + 1, 1) =  1.0;
        A(2 * i + 1, 2) =  px;
    }

    // Moore-Penrose pseudoinverse via JacobiSVD (stable for small matrices).
    A_pinv_ = A.completeOrthogonalDecomposition().pseudoInverse();

    RCLCPP_DEBUG(get_node()->get_logger(),
        "[OdomController] Kinematics pseudoinverse built (%ldx%ld).",
        A_pinv_.rows(), A_pinv_.cols());
}

std::vector<std::string>
RoverOdometryController::steer_state_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : steer_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}


std::vector<std::string>
RoverOdometryController::drive_state_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : drive_joint_names_) {
        // Drive encoder position (accumulated rad) — not velocity.
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}


bool RoverOdometryController::assign_interfaces()
{
    auto find_state = [this](const std::string & full_name)
        -> hardware_interface::LoanedStateInterface *
    {
        for (auto & iface : state_interfaces_) {
            if (iface.get_name() == full_name) { return &iface; }
        }
        return nullptr;
    };

    SteerStateHandles steer;
    DriveStateHandles drive;

    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
        const std::string steer_name =
            steer_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

        auto * s = find_state(steer_name);
        if (!s) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[OdomController] Missing state interface: %s", steer_name.c_str());
            return false;
        }
        steer.position.emplace_back(*s);
    }

    for (std::size_t i = 0; i < ODOM_NUM_WHEELS; ++i) {
        const std::string drive_name =
            drive_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

        auto * d = find_state(drive_name);
        if (!d) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[OdomController] Missing state interface: %s", drive_name.c_str());
            return false;
        }
        drive.position.emplace_back(*d);
    }

    steer_handles_ = std::move(steer);
    drive_handles_ = std::move(drive);
    return true;
}


void RoverOdometryController::publish_odom(
    double vx, double vy, double wz,
    const rclcpp::Time & stamp)
{
    const double qz = std::sin(theta_ / 2.0);
    const double qw = std::cos(theta_ / 2.0);

    // Odometry message

    nav_msgs::msg::Odometry odom;
    odom.header.stamp            = stamp;
    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_link";

    odom.pose.pose.position.x    = x_;
    odom.pose.pose.position.y    = y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;

    odom.twist.twist.linear.x    = vx;
    odom.twist.twist.linear.y    = vy;
    odom.twist.twist.angular.z   = wz;

    // --- Covariance ---
    for (auto & c : odom.pose.covariance)  { c = 0.0; }
    for (auto & c : odom.twist.covariance) { c = 0.0; }

    odom.pose.covariance[0]  = 0.01;   // var(x)
    odom.pose.covariance[7]  = 0.01;   // var(y)
    odom.pose.covariance[14] = 1e6;    // var(z)
    odom.pose.covariance[21] = 1e6;    // var(roll)
    odom.pose.covariance[28] = 1e6;    // var(pitch)
    odom.pose.covariance[35] = 0.02;   // var(yaw)

    odom.twist.covariance[0]  = 0.02;  // var(vx)
    odom.twist.covariance[7]  = 0.02;  // var(vy)
    odom.twist.covariance[14] = 1e6;   // var(vz)
    odom.twist.covariance[21] = 1e6;   // var(v_roll)
    odom.twist.covariance[28] = 1e6;   // var(v_pitch)
    odom.twist.covariance[35] = 0.03;  // var(wz)

    odom_pub_->publish(odom);

    // 2. Broadcast the Dynamic TF Transform Frame
    if (!tf_broadcaster_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
    }

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp            = stamp;
    odom_tf.header.frame_id         = "odom";
    odom_tf.child_frame_id          = "base_link";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    
    odom_tf.transform.rotation.z    = qz;
    odom_tf.transform.rotation.w    = qw;

    tf_broadcaster_->sendTransform(odom_tf);
}

}  // namespace rover_controller