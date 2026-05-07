// arm_teleop_node.cpp
//
// VR teleoperation controller for Husky A200 dual UR5e arms (Servo + position-error version).
//
// Subscribes to PoseStamped delta messages from Unity (one per arm) and
// commands each arm via MoveIt Servo using TwistStamped velocity messages.
// Gripper still goes through MoveGroupInterface since those are small
// discrete moves.
//
// Anchor-based absolute control:
//   - Unity sends header.frame_id == "grip_start" with zero pose -> we
//     snapshot the arm's current end-effector pose as the anchor.
//   - Subsequent messages carry the user's TOTAL hand offset since grip_start.
//     We compute the target pose (anchor + offset), then derive a velocity
//     by computing position error vs the arm's CURRENT EE pose, multiplied
//     by a P-gain. This is a simple proportional position controller built
//     on top of Servo's velocity interface.
//   - Unity sends header.frame_id == "grip_end" -> we publish a zero twist
//     and stop accepting deltas until the next grip_start.
//
// Why position-error instead of (target - last_target) / dt:
//   With dt-based twist, each VR frame produces a twist that Servo applies
//   for ONE Servo cycle (5ms), so the arm only covers ~15% of the distance
//   the operator's hand covered in that VR frame. Result: arm always lags
//   behind hand position.
//
//   With position-error twist, the arm continuously chases the target. As
//   long as the operator holds their hand out, the arm keeps moving toward
//   that hand position. When the arm catches up, the error shrinks and the
//   velocity tapers to zero. Position-faithful by construction.

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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Pose = geometry_msgs::msg::Pose;
using Float64 = std_msgs::msg::Float64;
using Trigger = std_srvs::srv::Trigger;
using std::placeholders::_1;


// P-gains for the position-error -> velocity conversion.
// Higher = more aggressive chasing of the target, more responsive but more
// likely to oscillate. Lower = sluggish but stable.
// Start with these values; raise GAIN_LINEAR if the arm feels slow to catch up.
constexpr double GAIN_LINEAR = 3.0;   // 1/s (so 1cm error -> 3cm/s velocity)
constexpr double GAIN_ANGULAR = 4.0;  // 1/s

// Hard caps on the velocity we publish, regardless of how big the error is.
// These dominate when the operator suddenly moves their hand a long way.
// Servo will also clamp at its scale.linear / scale.rotational params, but
// pre-clamping here gives us cleaner numbers.
constexpr double MAX_LINEAR_VEL = 1.5;   // m/s
constexpr double MAX_ANGULAR_VEL = 3.0;  // rad/s


class ArmController
{
public:
    ArmController(
        std::shared_ptr<rclcpp::Node> node,
        const std::string & arm_group_name,
        const std::string & gripper_group_name,
        const std::string & pose_topic,
        const std::string & gripper_topic,
        const std::string & servo_namespace,    // e.g. "/servo_node_arm_0"
        const std::string & ee_frame,           // e.g. "arm_0_tool0"
        double velocity_scale,
        double accel_scale)
    : node_(node),
      arm_group_name_(arm_group_name),
      gripper_group_name_(gripper_group_name),
      pose_topic_(pose_topic),
      gripper_topic_(gripper_topic),
      servo_namespace_(servo_namespace),
      ee_frame_(ee_frame),
      anchor_set_(false),
      gripper_busy_(false)
    {
        // Use MoveGroupInterface only for the gripper, for the initial pose
        // snapshot at grip_start, and for reading the arm's current EE pose
        // every delta callback. Streaming arm motion goes through Servo.
        arm_ = std::make_shared<MoveGroupInterface>(node_, arm_group_name_);
        arm_->setMaxVelocityScalingFactor(velocity_scale);
        arm_->setMaxAccelerationScalingFactor(accel_scale);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, gripper_group_name_);
        gripper_->setMaxVelocityScalingFactor(velocity_scale);
        gripper_->setMaxAccelerationScalingFactor(accel_scale);

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

        // Reentrant callback group so callbacks don't deadlock when reading current state.
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

        // Publisher for Servo's TwistStamped input topic.
        twist_pub_ = node_->create_publisher<TwistStamped>(
            servo_namespace_ + "/delta_twist_cmds", 10);

        // Start Servo on init. If Servo node hasn't come up yet we retry briefly.
        startServoAsync();

        RCLCPP_INFO(node_->get_logger(),
            "[%s] Ready. pose=%s gripper=%s twist=%s/delta_twist_cmds",
            arm_group_name_.c_str(),
            pose_topic_.c_str(),
            gripper_topic_.c_str(),
            servo_namespace_.c_str());
    }

private:
    // -----------------------------------------------------------------
    // Servo lifecycle: call /servo_node_X/start_servo so Servo accepts twists.
    // -----------------------------------------------------------------
    void startServoAsync()
    {
        auto client = node_->create_client<Trigger>(
            servo_namespace_ + "/start_servo");

        std::thread([this, client]() {
            for (int i = 0; i < 30; ++i) {
                if (client->wait_for_service(std::chrono::seconds(1))) {
                    auto req = std::make_shared<Trigger::Request>();
                    auto fut = client->async_send_request(req);
                    RCLCPP_INFO(node_->get_logger(),
                        "[%s] start_servo request sent.",
                        arm_group_name_.c_str());
                    return;
                }
            }
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] start_servo service never appeared. Servo node not running?",
                arm_group_name_.c_str());
        }).detach();
    }

    // -----------------------------------------------------------------
    // Compute a Cartesian twist proportional to the position error between
    // the target pose and the arm's current pose.
    //
    //   linear_velocity  = (target_pos - current_pos) * GAIN_LINEAR
    //   angular_velocity = axis_angle(target_orient * inv(current_orient))
    //                       * GAIN_ANGULAR
    //
    // Then clamp to MAX_LINEAR_VEL / MAX_ANGULAR_VEL.
    // -----------------------------------------------------------------
    void computeTwistFromError(
        const Pose & target,
        const Pose & current,
        TwistStamped & out)
    {
        // ---- Linear: position error * gain ----
        double ex = target.position.x - current.position.x;
        double ey = target.position.y - current.position.y;
        double ez = target.position.z - current.position.z;

        double vx = ex * GAIN_LINEAR;
        double vy = ey * GAIN_LINEAR;
        double vz = ez * GAIN_LINEAR;

        double v_mag = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (v_mag > MAX_LINEAR_VEL) {
            double s = MAX_LINEAR_VEL / v_mag;
            vx *= s; vy *= s; vz *= s;
        }

        out.twist.linear.x = vx;
        out.twist.linear.y = vy;
        out.twist.linear.z = vz;

        // ---- Angular: orientation error as axis * angle * gain ----
        tf2::Quaternion q_current(
            current.orientation.x, current.orientation.y,
            current.orientation.z, current.orientation.w);
        tf2::Quaternion q_target(
            target.orientation.x, target.orientation.y,
            target.orientation.z, target.orientation.w);

        // q_diff = q_target * inv(q_current). This is the rotation that
        // takes the arm from current orientation to target orientation.
        tf2::Quaternion q_diff = q_target * q_current.inverse();
        q_diff.normalize();

        // Force shortest path (avoid going the long way around).
        if (q_diff.w() < 0.0) {
            q_diff.setValue(-q_diff.x(), -q_diff.y(), -q_diff.z(), -q_diff.w());
        }

        double angle = 2.0 * std::acos(std::min(1.0, std::max(-1.0, q_diff.w())));
        double sin_half = std::sqrt(1.0 - q_diff.w() * q_diff.w());

        double wx, wy, wz;
        if (sin_half < 1e-6) {
            // Almost no rotation error.
            wx = wy = wz = 0.0;
        } else {
            double ax = q_diff.x() / sin_half;
            double ay = q_diff.y() / sin_half;
            double az = q_diff.z() / sin_half;
            double w_mag = angle * GAIN_ANGULAR;

            if (w_mag > MAX_ANGULAR_VEL) {
                w_mag = MAX_ANGULAR_VEL;
            }

            wx = ax * w_mag;
            wy = ay * w_mag;
            wz = az * w_mag;
        }

        out.twist.angular.x = wx;
        out.twist.angular.y = wy;
        out.twist.angular.z = wz;
    }

    // -----------------------------------------------------------------
    // Publish a zero twist. Used on grip_end to bring the arm to rest cleanly.
    // -----------------------------------------------------------------
    void publishZeroTwist()
    {
        TwistStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "base_link";
        // All twist components default to 0.
        twist_pub_->publish(msg);
    }

    // -----------------------------------------------------------------
    // Pose callback. Called whenever Unity publishes a pose update for this arm.
    // -----------------------------------------------------------------
    void poseCallback(const PoseStamped::SharedPtr msg)
    {
        const std::string & frame = msg->header.frame_id;

        // ---- grip_start: snapshot anchor pose ----
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

        // ---- grip_end: stop arm by publishing zero twist ----
        if (frame == "grip_end") {
            {
                std::lock_guard<std::mutex> lock(anchor_mutex_);
                anchor_set_ = false;
            }
            publishZeroTwist();
            RCLCPP_INFO(node_->get_logger(),
                "[%s] grip_end.", arm_group_name_.c_str());
            return;
        }

        // ---- delta: compute target pose, derive twist from position error ----
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

        // Compute absolute target pose: anchor + Unity-reported offset.
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

        // Read the arm's CURRENT EE pose (from CurrentStateMonitor's local cache).
        Pose current;
        try {
            current = arm_->getCurrentPose().pose;
        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[%s] getCurrentPose failed: %s",
                arm_group_name_.c_str(), e.what());
            return;
        }

        // Build twist from the error (target - current).
        TwistStamped twist_msg;
        twist_msg.header.stamp = node_->now();
        twist_msg.header.frame_id = "base_link";
        computeTwistFromError(target, current, twist_msg);

        twist_pub_->publish(twist_msg);
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
    std::string servo_namespace_;
    std::string ee_frame_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<Float64>::SharedPtr gripper_sub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    std::mutex anchor_mutex_;
    Pose anchor_pose_;
    bool anchor_set_;

    std::atomic<bool> gripper_busy_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("arm_teleop_node");

    const double velocity_scale = 0.3;
    const double accel_scale = 0.3;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ArmController arm_0(
        node, "arm_0", "arm_0_gripper",
        "/unity/arm_0_target_pose", "/unity/gripper_0_cmd",
        "/servo_node_arm_0", "arm_0_tool0",
        velocity_scale, accel_scale);

    ArmController arm_1(
        node, "arm_1", "arm_1_gripper",
        "/unity/arm_1_target_pose", "/unity/gripper_1_cmd",
        "/servo_node_arm_1", "arm_1_tool0",
        velocity_scale, accel_scale);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}