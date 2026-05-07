// arm_teleop_node.cpp
//
// VR teleoperation controller for Husky A200 dual UR5e arms.
//
// Subscribes to PoseStamped delta messages from Unity (one per arm) and
// uses MoveIt to drive the arm via computeCartesianPath. Also handles
// gripper open/close from Float64 messages.
//
// Anchor-based absolute control:
//   - Unity sends header.frame_id == "grip_start" with zero pose -> we
//     snapshot the arm's current end-effector pose as the anchor.
//   - Subsequent messages carry the user's TOTAL hand offset since grip_start.
//     We add that offset to the anchor to get the absolute target pose.
//   - Unity sends header.frame_id == "grip_end" -> we stop accepting deltas
//     until the next grip_start.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using Float64 = std_msgs::msg::Float64;
using std::placeholders::_1;


class ArmController
{
public:
    ArmController(
        std::shared_ptr<rclcpp::Node> node,
        const std::string & arm_group_name,
        const std::string & gripper_group_name,
        const std::string & pose_topic,
        const std::string & gripper_topic,
        double velocity_scale,
        double accel_scale,
        double cartesian_eef_step,
        double cartesian_min_fraction)
    : node_(node),
      arm_group_name_(arm_group_name),
      gripper_group_name_(gripper_group_name),
      pose_topic_(pose_topic),
      gripper_topic_(gripper_topic),
      cartesian_eef_step_(cartesian_eef_step),
      cartesian_min_fraction_(cartesian_min_fraction),
      anchor_set_(false),
      planning_busy_(false),
      gripper_busy_(false)
    {
        arm_ = std::make_shared<MoveGroupInterface>(node_, arm_group_name_);
        arm_->setMaxVelocityScalingFactor(velocity_scale);
        arm_->setMaxAccelerationScalingFactor(accel_scale);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, gripper_group_name_);
        gripper_->setMaxVelocityScalingFactor(velocity_scale);
        gripper_->setMaxAccelerationScalingFactor(accel_scale);

        // Force CurrentStateMonitor to start NOW and wait for first joint state.
        // Without this, MGI lazy-creates the monitor on first getCurrentPose()
        // call which races with joint_states publishing.
        if (!arm_->startStateMonitor(5.0)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] startStateMonitor timed out for arm.",
                arm_group_name_.c_str());
        }
        if (!gripper_->startStateMonitor(5.0)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] startStateMonitor timed out for gripper.",
                arm_group_name_.c_str());
        }


        // Reentrant callback group: lets this callback call getCurrentState()
        // without deadlocking the executor (the executor can process incoming
        // joint_state messages on another thread while we're inside the callback).
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;

        pose_sub_ = node_->create_subscription<PoseStamped>(
            pose_topic_, 10,
            std::bind(&ArmController::poseCallback, this, _1),
            sub_opts);

        gripper_sub_ = node_->create_subscription<Float64>(
            gripper_topic_, 10,
            std::bind(&ArmController::gripperCallback, this, _1),
            sub_opts);
        RCLCPP_INFO(node_->get_logger(),
            "[%s] Ready. pose=%s gripper=%s",
            arm_group_name_.c_str(), pose_topic_.c_str(), gripper_topic_.c_str());
    }

private:
    void poseCallback(const PoseStamped::SharedPtr msg)
    {
        const std::string & frame = msg->header.frame_id;

        if (frame == "grip_start") {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            try {
                auto current_state = arm_->getCurrentState(2.0);
                if (!current_state) {
                    RCLCPP_ERROR(node_->get_logger(),
                        "[%s] grip_start: no fresh robot state.",
                        arm_group_name_.c_str());
                    anchor_set_ = false;
                    return;
                }
                anchor_pose_ = arm_->getCurrentPose().pose;
                anchor_set_ = true;
                RCLCPP_INFO(node_->get_logger(),
                    "[%s] grip_start. Anchor at (%.3f, %.3f, %.3f)",
                    arm_group_name_.c_str(),
                    anchor_pose_.position.x,
                    anchor_pose_.position.y,
                    anchor_pose_.position.z);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(node_->get_logger(),
                    "[%s] grip_start failed to read current pose: %s",
                    arm_group_name_.c_str(), e.what());
                anchor_set_ = false;
            }
            return;
        }

        if (frame == "grip_end") {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            anchor_set_ = false;
            RCLCPP_INFO(node_->get_logger(),
                "[%s] grip_end.", arm_group_name_.c_str());
            return;
        }

        Pose anchor_copy;
        bool have_anchor;
        {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            have_anchor = anchor_set_;
            anchor_copy = anchor_pose_;
        }

        if (!have_anchor) {
            return;
        }

        bool busy_expected = false;
        if (!planning_busy_.compare_exchange_strong(busy_expected, true)) {
            return;
        }

        Pose target;
        target.position.x = anchor_copy.position.x + msg->pose.position.x;
        target.position.y = anchor_copy.position.y + msg->pose.position.y;
        target.position.z = anchor_copy.position.z + msg->pose.position.z;

        tf2::Quaternion q_anchor(
            anchor_copy.orientation.x,
            anchor_copy.orientation.y,
            anchor_copy.orientation.z,
            anchor_copy.orientation.w);
        tf2::Quaternion q_delta(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Quaternion q_target = q_delta * q_anchor;
        q_target.normalize();

        target.orientation.x = q_target.x();
        target.orientation.y = q_target.y();
        target.orientation.z = q_target.z();
        target.orientation.w = q_target.w();

        std::vector<Pose> waypoints;
        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;
        arm_->setStartStateToCurrentState();
        double fraction = arm_->computeCartesianPath(
            waypoints, cartesian_eef_step_, 0.0, trajectory);

        if (fraction >= cartesian_min_fraction_) {
            arm_->asyncExecute(trajectory);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[%s] Cartesian path only achieved %.2f%%, skipping execute.",
                arm_group_name_.c_str(), fraction * 100.0);
        }

        planning_busy_ = false;
    }

    void gripperCallback(const Float64::SharedPtr msg)
    {
        bool busy_expected = false;
        if (!gripper_busy_.compare_exchange_strong(busy_expected, true)) {
            return;
        }

        std::vector<double> target = {msg->data};
        gripper_->setStartStateToCurrentState();
        gripper_->setJointValueTarget(target);

        MoveGroupInterface::Plan plan;
        bool ok = (gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (ok) {
            gripper_->asyncExecute(plan);
            RCLCPP_INFO(node_->get_logger(),
                "[%s gripper] -> %.3f rad", arm_group_name_.c_str(), msg->data);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[%s gripper] plan failed for value %.3f",
                arm_group_name_.c_str(), msg->data);
        }

        gripper_busy_ = false;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::string pose_topic_;
    std::string gripper_topic_;

    double cartesian_eef_step_;
    double cartesian_min_fraction_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<Float64>::SharedPtr gripper_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::mutex anchor_mutex_;
    Pose anchor_pose_;
    bool anchor_set_;

    std::atomic<bool> planning_busy_;
    std::atomic<bool> gripper_busy_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("arm_teleop_node");

    const double velocity_scale = 0.3;
    const double accel_scale = 0.3;
    const double cartesian_eef_step = 0.01;
    const double cartesian_min_fraction = 0.85;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // Give the spinner time to start processing before constructing MoveGroupInterfaces.
    // This avoids a race where MGI tries to read joint states before the executor is
    // actually spinning.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ArmController arm_0(
        node, "arm_0", "arm_0_gripper",
        "/unity/arm_0_target_pose", "/unity/gripper_0_cmd",
        velocity_scale, accel_scale,
        cartesian_eef_step, cartesian_min_fraction);

    ArmController arm_1(
        node, "arm_1", "arm_1_gripper",
        "/unity/arm_1_target_pose", "/unity/gripper_1_cmd",
        velocity_scale, accel_scale,
        cartesian_eef_step, cartesian_min_fraction);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}