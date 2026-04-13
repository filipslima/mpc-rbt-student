#include <cmath>

#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here
    x_ = -0.5;
    y_ = 0.0;
    theta_ = 0.0;

    odometry_.pose.pose.position.x = x_;//+ M_PI / 2.0
    odometry_.pose.pose.position.y = y_;
    odometry_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    q.normalize();
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    // Subscriber for joint_states
    // add code here
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // Publisher for odometry
    // add code here
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    if (msg.velocity.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "JointState velocity does not contain 2 values.");
        return;
    }

    if (!std::isfinite(msg.velocity[0]) || !std::isfinite(msg.velocity[1])) {
        RCLCPP_WARN(this->get_logger(), "JointState velocity contains NaN or Inf.");
        return;
    }

    auto current_time = this->get_clock()->now();

    if (first_message_) {
        last_time_ = current_time;
        first_message_ = false;
        return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (!std::isfinite(dt) || dt <= 0.0) {
        return;
    }

    double left_wheel_vel = msg.velocity[1];
    double right_wheel_vel = msg.velocity[0];

    updateOdometry(left_wheel_vel, right_wheel_vel, dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    const double r = robot_config::WHEEL_RADIUS;
    const double half_wheel_base = robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    double linear = r * (right_wheel_vel + left_wheel_vel) / 2.0;
    double angular = r * (right_wheel_vel - left_wheel_vel) / (2.0 * half_wheel_base);

    if (!std::isfinite(linear) || !std::isfinite(angular)) {
        RCLCPP_WARN(this->get_logger(), "Computed linear/angular velocity is NaN or Inf.");
        return;
    }

    //double theta_mid = theta_ + angular * dt / 2.0;

    //x_ += linear * std::cos(theta_mid) * dt;
    //y_ += linear * std::sin(theta_mid) * dt;

    x_ += linear * std::cos(theta_) * dt;
    y_ += linear * std::sin(theta_) * dt;

    theta_ += angular * dt;

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(theta_)) {
        RCLCPP_WARN(this->get_logger(), "Pose became NaN or Inf.");
        return;
    }

    odometry_.pose.pose.position.x = x_;
    odometry_.pose.pose.position.y = y_;
    odometry_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    q.normalize();
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;

    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    // add code here
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // add code here
    
    // ********
    // * Help *
    // ********
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
