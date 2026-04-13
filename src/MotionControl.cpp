#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

MotionControlNode::MotionControlNode()
    : rclcpp::Node("motion_control_node"),
      max_linear_velocity_(0.05),
      max_angular_velocity_(0.1),
      lookahead_distance_(0.3),
      goal_tolerance_(0.2),
      collision_distance_threshold_(0.4),
      navigation_active_(false),
      collision_detected_(false),
      odom_received_(false),
      path_received_(false)
{
    // Subscribers for odometry and laser scans
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&MotionControlNode::odomCallback, this, _1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/tiago_base/Hokuyo_URG_04LX_UG01", 10,
        std::bind(&MotionControlNode::lidarCallback, this, _1));

    // Publisher for robot control
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Client for path planning
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    // Action server
    nav_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, _1, _2),
        std::bind(&MotionControlNode::navHandleCancel, this, _1),
        std::bind(&MotionControlNode::navHandleAccepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Motion control node started.");

    // Connect to path planning service server
    while (!plan_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for planner service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for planner service /plan ...");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to planner service.");
}

void MotionControlNode::publishZeroTwist()
{
    geometry_msgs::msg::Twist stop;
    twist_publisher_->publish(stop);
}

double MotionControlNode::getYawFromPose(const geometry_msgs::msg::Pose & pose) const
{
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double MotionControlNode::distanceToGoal() const
{
    const double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    const double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

void MotionControlNode::checkCollision()
{
    collision_detected_ = false;

    if (!navigation_active_ || laser_scan_.ranges.empty()) {
        return;
    }

    const double sector_half_angle = 35.0 * M_PI / 180.0;

    const double forward_angle = 0.0;

    float min_range = std::numeric_limits<float>::infinity();
    float global_min = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        const double angle =
            laser_scan_.angle_min + static_cast<double>(i) * laser_scan_.angle_increment;

        const float range = laser_scan_.ranges[i];

        if (!std::isfinite(range)) {
            continue;
        }

        if (range < laser_scan_.range_min || range > laser_scan_.range_max) {
            continue;
        }

        if (range < global_min) {
            global_min = range;
        }

        if (std::abs(angle - forward_angle) > sector_half_angle) {
            continue;
        }

        if (range < min_range) {
            min_range = range;
        }
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "global_min = %.3f, sector_min = %.3f", global_min, min_range);

    if (min_range < collision_distance_threshold_) {
        collision_detected_ = true;

        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);

        if (navigation_active_ && goal_handle_) {
            auto result = std::make_shared<NavigateToPose::Result>();
            navigation_active_ = false;
            path_received_ = false;

            RCLCPP_WARN(
                this->get_logger(),
                "Collision risk detected: min_range = %.3f m. Navigation aborted.",
                min_range);

            goal_handle_->abort(result);
        }
    }
}

void MotionControlNode::updateTwist()
{
    if (!navigation_active_ || !path_received_ || path_.poses.empty() || !odom_received_) {
        return;
    }

    if (collision_detected_) {
        publishZeroTwist();
        return;
    }

    if (distanceToGoal() < goal_tolerance_) {
        publishZeroTwist();

        if (goal_handle_) {
            auto result = std::make_shared<NavigateToPose::Result>();
            navigation_active_ = false;
            path_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            goal_handle_->succeed(result);
        }
        return;
    }

    const double robot_x = current_pose_.pose.position.x;
    const double robot_y = current_pose_.pose.position.y;
    const double robot_yaw = getYawFromPose(current_pose_.pose);

    // Najdi nejbližší bod na trase
    size_t nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.poses.size(); ++i) {
        const double dx = path_.poses[i].pose.position.x - robot_x;
        const double dy = path_.poses[i].pose.position.y - robot_y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = i;
        }
    }

    // Vezmi bod o pár kroků dál, ne úplný konec
    size_t target_idx = std::min(nearest_idx + 5, path_.poses.size() - 1);
    const auto & target_pose = path_.poses[target_idx];

    const double dx = target_pose.pose.position.x - robot_x;
    const double dy = target_pose.pose.position.y - robot_y;

    const double target_heading = std::atan2(dy, dx);

    double alpha = target_heading - robot_yaw;
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;

    double v = max_linear_velocity_;
    double omega = 2.0 * alpha;

    // Když je robot hodně špatně natočený, skoro zastav a nejdřív se srovnej
    if (std::abs(alpha) > 0.6) {
        v = 0.0;
    } else if (std::abs(alpha) > 0.3) {
        v = 0.02;
    }

    // malá mrtvá zóna
    if (std::abs(alpha) < 0.03) {
        omega = 0.0;
    }

    twist.linear.x = std::clamp(v, 0.0, max_linear_velocity_);
    twist.angular.z = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

    RCLCPP_INFO(this->get_logger(),
        "nearest=%zu target=%zu target=(%.2f, %.2f) robot=(%.2f, %.2f) yaw=%.2f alpha=%.2f v=%.2f omega=%.2f",
        nearest_idx, target_idx,
        target_pose.pose.position.x, target_pose.pose.position.y,
        robot_x, robot_y, robot_yaw, alpha,
        twist.linear.x, twist.angular.z);

    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(
        this->get_logger(),
        "Received goal request: x=%.3f y=%.3f",
        goal->pose.pose.position.x,
        goal->pose.pose.position.y);

    if (!odom_received_) {
        RCLCPP_WARN(this->get_logger(), "No odometry yet, rejecting goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Received request to cancel navigation.");
    navigation_active_ = false;
    path_received_ = false;
    publishZeroTwist();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::requestPlan(const geometry_msgs::msg::PoseStamped & goal)
{
    auto request = std::make_shared<GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal;
    request->tolerance = 0.1;

    plan_client_->async_send_request(
        request,
        std::bind(&MotionControlNode::pathCallback, this, _1));
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;
    path_received_ = false;
    collision_detected_ = false;

    RCLCPP_INFO(this->get_logger(), "Goal accepted, requesting path.");
    requestPlan(goal_pose_);
}

void MotionControlNode::execute()
{
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    rclcpp::Rate loop_rate(5.0);

    while (rclcpp::ok() && navigation_active_) {
        if (!goal_handle_) {
            return;
        }

        if (goal_handle_->is_canceling()) {
            publishZeroTwist();
            auto result = std::make_shared<NavigateToPose::Result>();
            navigation_active_ = false;
            path_received_ = false;
            goal_handle_->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Navigation canceled.");
            return;
        }

        feedback->current_pose = current_pose_;
        feedback->distance_remaining = distanceToGoal();
        feedback->navigation_time = rclcpp::Duration::from_seconds(0.0);
        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
{
    auto response = future.get();

    if (!response || response->plan.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Planner returned empty path.");

        publishZeroTwist();
        navigation_active_ = false;
        path_received_ = false;

        if (goal_handle_) {
            auto result = std::make_shared<NavigateToPose::Result>();
            goal_handle_->abort(result);
        }
        return;
    }

    path_ = response->plan;
    path_received_ = true;
    navigation_active_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Received path with %zu poses. Starting execution.",
        path_.poses.size());

    std::thread(&MotionControlNode::execute, this).detach();
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    odom_received_ = true;

    
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_scan_ = *msg;
    checkCollision();
}