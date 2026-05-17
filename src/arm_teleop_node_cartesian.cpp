// arm_teleop_node_cartesian.cpp
//
//   global_translation = tool_anchor[:3] + rel_translation
//   global_quat = quat_multiply(rel_quat, tool_anchor[3:])
//   self.target[:3] += smooth_step * (global_translation - self.target[:3])
//   self.target[3:] = interpolate_quat(self.target[3:], global_quat, smooth_step)
//
// Tunables (CLI):
//   position_scale       : default 0.5 .
//                          1.0 = 1:1 
//   smooth_step          : default 0.1.
//                          0.05 = very smooth, more lag 
//                          0.5  = very responsive, less smooth.
//                          1.0  = no smoothing.
//   mirror_mode          : default false. Operator behind robot facing
//                          
//   operator_yaw_offset_deg : default 0.
//   translation_only     : default false. Lock orientation to anchor.
//   sign_x, sign_y, sign_z  : default +1.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>


using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using Float64 = std_msgs::msg::Float64;
using GripperCommand = control_msgs::action::GripperCommand;
using std::placeholders::_1;


static tf2::Matrix3x3 rpyMatrix(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::Matrix3x3(q);
}


static const tf2::Matrix3x3 ARM_0_FROM_CHASSIS =
    rpyMatrix(0.785398, 0.0, 3.14159).transpose();
static const tf2::Matrix3x3 ARM_1_FROM_CHASSIS =
    rpyMatrix(0.785398, 0.0, 0.0).transpose();


constexpr double TARGET_PUBLISH_RATE_HZ = 50.0;
constexpr double TARGET_PUBLISH_PERIOD = 1.0 / TARGET_PUBLISH_RATE_HZ;
constexpr double GRIPPER_MAX_EFFORT = 50.0;


struct TuneParams {
    double position_scale;
    double smooth_step;          // 0..1 exponential smoothing factor
    bool   translation_only;
    double sign_x;
    double sign_y;
    double sign_z;
    tf2::Matrix3x3 R_chassis_from_operator;
};



static tf2::Quaternion smoothQuat(
    const tf2::Quaternion & cur,
    const tf2::Quaternion & target,
    double step)
{
    // Pick the closer hemisphere (dot >= 0) to avoid the "long way around".
    double dot = cur.x() * target.x() + cur.y() * target.y() +
                 cur.z() * target.z() + cur.w() * target.w();
    tf2::Quaternion target_aligned = target;
    if (dot < 0.0) {
        target_aligned = tf2::Quaternion(
            -target.x(), -target.y(), -target.z(), -target.w());
    }
    tf2::Quaternion out(
        cur.x() + step * (target_aligned.x() - cur.x()),
        cur.y() + step * (target_aligned.y() - cur.y()),
        cur.z() + step * (target_aligned.z() - cur.z()),
        cur.w() + step * (target_aligned.w() - cur.w()));
    out.normalize();
    return out;
}


class ArmController
{
public:
    ArmController(
        std::shared_ptr<rclcpp::Node> node,
        const std::string & arm_name,
        const std::string & pose_topic,
        const std::string & gripper_topic,
        const std::string & target_frame_topic,
        const std::string & current_pose_topic,
        const std::string & gripper_action_topic,
        const std::string & base_link_name,
        const tf2::Matrix3x3 & R_arm_from_chassis,
        const TuneParams & params)
    : node_(node),
      arm_name_(arm_name),
      pose_topic_(pose_topic),
      gripper_topic_(gripper_topic),
      target_frame_topic_(target_frame_topic),
      current_pose_topic_(current_pose_topic),
      gripper_action_topic_(gripper_action_topic),
      base_link_name_(base_link_name),
      R_arm_from_chassis_(R_arm_from_chassis),
      params_(params),
      anchor_set_(false),
      have_current_pose_(false)
    {
        R_arm_from_operator_ = R_arm_from_chassis_ * params_.R_chassis_from_operator;
        R_arm_from_operator_.getRotation(q_arm_from_operator_);
        q_operator_from_arm_ = q_arm_from_operator_.inverse();

        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;

        current_pose_sub_ = node_->create_subscription<PoseStamped>(
            current_pose_topic_, 10,
            std::bind(&ArmController::currentPoseCallback, this, _1),
            sub_opts);

        pose_sub_ = node_->create_subscription<PoseStamped>(
            pose_topic_, 10,
            std::bind(&ArmController::poseCallback, this, _1),
            sub_opts);

        gripper_sub_ = node_->create_subscription<Float64>(
            gripper_topic_, 10,
            std::bind(&ArmController::gripperCallback, this, _1),
            sub_opts);

        target_pub_ = node_->create_publisher<PoseStamped>(
            target_frame_topic_, 10);

        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            node_, gripper_action_topic_, callback_group_);

        target_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(TARGET_PUBLISH_PERIOD),
            std::bind(&ArmController::publishTargetTick, this),
            callback_group_);

        RCLCPP_INFO(node_->get_logger(),
            "[%s] %s -> %s",
            arm_name_.c_str(), pose_topic_.c_str(), target_frame_topic_.c_str());
    }

private:
    void currentPoseCallback(const PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_);
        current_pose_ = msg->pose;
        have_current_pose_ = true;
    }

    void poseCallback(const PoseStamped::SharedPtr msg)
    {
        const std::string & frame = msg->header.frame_id;

        if (frame == "grip_start") {
            
            Pose anchor;
            bool ok;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                anchor = current_pose_;
                ok = have_current_pose_;
            }
            if (!ok) {
                RCLCPP_WARN(node_->get_logger(),
                    "[%s] grip_start before current_pose available",
                    arm_name_.c_str());
                return;
            }
            {
                std::lock_guard<std::mutex> lock(anchor_mutex_);
                anchor_pose_ = anchor;
                smoothed_target_ = anchor;
                anchor_set_ = true;
            }
            RCLCPP_INFO(node_->get_logger(),
                "[%s] grip_start anchor=(%.3f, %.3f, %.3f)",
                arm_name_.c_str(),
                anchor.position.x, anchor.position.y, anchor.position.z);
            return;
        }

        if (frame == "grip_end") {
            {
                std::lock_guard<std::mutex> lock(anchor_mutex_);
                anchor_set_ = false;
            }
            return;
        }

        Pose anchor_copy;
        Pose smoothed_target_copy;
        bool have_anchor;
        {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            have_anchor = anchor_set_;
            anchor_copy = anchor_pose_;
            smoothed_target_copy = smoothed_target_;
        }
        if (!have_anchor) return;

        // ---- Raw target from delta (no filter on input) ----
        tf2::Vector3 dp_op(
            params_.sign_x * msg->pose.position.x * params_.position_scale,
            params_.sign_y * msg->pose.position.y * params_.position_scale,
            params_.sign_z * msg->pose.position.z * params_.position_scale);
        tf2::Vector3 dp_arm = R_arm_from_operator_ * dp_op;

        const double raw_x = anchor_copy.position.x + dp_arm.x();
        const double raw_y = anchor_copy.position.y + dp_arm.y();
        const double raw_z = anchor_copy.position.z + dp_arm.z();

        tf2::Quaternion q_anchor(
            anchor_copy.orientation.x, anchor_copy.orientation.y,
            anchor_copy.orientation.z, anchor_copy.orientation.w);

        tf2::Quaternion q_raw_target;
        if (params_.translation_only) {
            q_raw_target = q_anchor;
        } else {
            tf2::Quaternion q_delta_op(
                msg->pose.orientation.x, msg->pose.orientation.y,
                msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Quaternion q_delta_arm =
                q_arm_from_operator_ * q_delta_op * q_operator_from_arm_;
            q_raw_target = q_delta_arm * q_anchor;
            q_raw_target.normalize();
        }

        // ---- Exponential smoothing on target  ----
        const double s = params_.smooth_step;
        const double new_x = smoothed_target_copy.position.x +
                             s * (raw_x - smoothed_target_copy.position.x);
        const double new_y = smoothed_target_copy.position.y +
                             s * (raw_y - smoothed_target_copy.position.y);
        const double new_z = smoothed_target_copy.position.z +
                             s * (raw_z - smoothed_target_copy.position.z);

        tf2::Quaternion q_cur(
            smoothed_target_copy.orientation.x, smoothed_target_copy.orientation.y,
            smoothed_target_copy.orientation.z, smoothed_target_copy.orientation.w);
        tf2::Quaternion q_new = smoothQuat(q_cur, q_raw_target, s);

        Pose new_smoothed;
        new_smoothed.position.x = new_x;
        new_smoothed.position.y = new_y;
        new_smoothed.position.z = new_z;
        new_smoothed.orientation.x = q_new.x();
        new_smoothed.orientation.y = q_new.y();
        new_smoothed.orientation.z = q_new.z();
        new_smoothed.orientation.w = q_new.w();

        {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            smoothed_target_ = new_smoothed;
        }
    }

    void publishTargetTick()
    {
        Pose to_publish;
        bool should_publish;
        {
            std::lock_guard<std::mutex> lock(anchor_mutex_);
            should_publish = anchor_set_;
            to_publish = smoothed_target_;
        }
        if (!should_publish) return;

        PoseStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = base_link_name_;
        msg.pose = to_publish;
        target_pub_->publish(msg);
    }

    void gripperCallback(const Float64::SharedPtr msg)
    {
        if (!gripper_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[%s gripper] action server not ready", arm_name_.c_str());
            return;
        }
        GripperCommand::Goal goal;
        goal.command.position = msg->data;
        goal.command.max_effort = GRIPPER_MAX_EFFORT;
        gripper_client_->async_send_goal(goal);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string arm_name_;
    std::string pose_topic_;
    std::string gripper_topic_;
    std::string target_frame_topic_;
    std::string current_pose_topic_;
    std::string gripper_action_topic_;
    std::string base_link_name_;
    tf2::Matrix3x3 R_arm_from_chassis_;
    tf2::Matrix3x3 R_arm_from_operator_;
    tf2::Quaternion q_arm_from_operator_;
    tf2::Quaternion q_operator_from_arm_;
    TuneParams params_;

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<Float64>::SharedPtr gripper_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    std::mutex anchor_mutex_;
    Pose anchor_pose_;
    Pose smoothed_target_;
    bool anchor_set_;

    std::mutex current_pose_mutex_;
    Pose current_pose_;
    bool have_current_pose_;
};


static std::string resolve_namespace(const rclcpp::Node::SharedPtr & node)
{
    std::string ns = node->declare_parameter<std::string>("robot_namespace", "");
    if (ns.empty()) {
        ns = node->get_namespace();
        if (ns == "/") ns = "";
    }
    return ns;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("arm_teleop_node");

    const std::string ns = resolve_namespace(node);

    TuneParams params;
    params.position_scale     = node->declare_parameter<double>("position_scale", 0.5);
    params.smooth_step        = node->declare_parameter<double>("smooth_step", 0.1);
    params.translation_only   = node->declare_parameter<bool>("translation_only", false);
    params.sign_x             = node->declare_parameter<double>("sign_x", 1.0);
    params.sign_y             = node->declare_parameter<double>("sign_y", 1.0);
    params.sign_z             = node->declare_parameter<double>("sign_z", 1.0);
    const bool mirror_mode    = node->declare_parameter<bool>("mirror_mode", false);
    const double yaw_offset_deg =
        node->declare_parameter<double>("operator_yaw_offset_deg", 0.0);

    if (params.smooth_step <= 0.0) params.smooth_step = 0.001;
    if (params.smooth_step > 1.0) params.smooth_step = 1.0;

    const double yaw_rad = yaw_offset_deg * M_PI / 180.0;
    params.R_chassis_from_operator = rpyMatrix(0.0, 0.0, yaw_rad);

    RCLCPP_INFO(node->get_logger(),
        "ns='%s'  scale=%.2f  smooth=%.2f  trans_only=%s  mirror=%s  yaw=%.1f  signs=(%.0f,%.0f,%.0f)",
        ns.c_str(), params.position_scale, params.smooth_step,
        params.translation_only ? "true" : "false",
        mirror_mode ? "true" : "false",
        yaw_offset_deg,
        params.sign_x, params.sign_y, params.sign_z);

    const std::string cmc_arm_0 = ns + "/manipulators/cartesian_motion_controller_arm_0";
    const std::string cmc_arm_1 = ns + "/manipulators/cartesian_motion_controller_arm_1";
    const std::string gripper_arm_0 = ns + "/manipulators/arm_0_gripper_controller/gripper_cmd";
    const std::string gripper_arm_1 = ns + "/manipulators/arm_1_gripper_controller/gripper_cmd";

    const std::string left_pose_topic    = mirror_mode ? "/unity/arm_1_target_pose"
                                                        : "/unity/arm_0_target_pose";
    const std::string left_gripper_topic = mirror_mode ? "/unity/gripper_1_cmd"
                                                        : "/unity/gripper_0_cmd";
    const std::string right_pose_topic    = mirror_mode ? "/unity/arm_0_target_pose"
                                                         : "/unity/arm_1_target_pose";
    const std::string right_gripper_topic = mirror_mode ? "/unity/gripper_0_cmd"
                                                         : "/unity/gripper_1_cmd";

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ArmController left_controller(
        node, mirror_mode ? "LTouch->arm_1" : "LTouch->arm_0",
        left_pose_topic, left_gripper_topic,
        (mirror_mode ? cmc_arm_1 : cmc_arm_0) + "/target_frame",
        (mirror_mode ? cmc_arm_1 : cmc_arm_0) + "/current_pose",
        mirror_mode ? gripper_arm_1 : gripper_arm_0,
        mirror_mode ? "arm_1_base_link" : "arm_0_base_link",
        mirror_mode ? ARM_1_FROM_CHASSIS : ARM_0_FROM_CHASSIS,
        params);

    ArmController right_controller(
        node, mirror_mode ? "RTouch->arm_0" : "RTouch->arm_1",
        right_pose_topic, right_gripper_topic,
        (mirror_mode ? cmc_arm_0 : cmc_arm_1) + "/target_frame",
        (mirror_mode ? cmc_arm_0 : cmc_arm_1) + "/current_pose",
        mirror_mode ? gripper_arm_0 : gripper_arm_1,
        mirror_mode ? "arm_0_base_link" : "arm_1_base_link",
        mirror_mode ? ARM_0_FROM_CHASSIS : ARM_1_FROM_CHASSIS,
        params);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
