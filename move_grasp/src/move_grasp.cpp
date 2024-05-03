#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <move_grasp_msg/srv/move_to.hpp>
#include <move_grasp_msg/srv/get_end_effector_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MoveItNode : public rclcpp::Node
{
public:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

public:
    MoveItNode(const std::string &node_name)
        : Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        this->declare_parameter("planning_group", "ur_manipulator");
        planning_group_ = this->get_parameter("planning_group").as_string();
        RCLCPP_INFO(get_logger(), "Planning group: %s", planning_group_.c_str());
    }

    void Init()
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    }

    void SetTargetByName(const std::string &target)
    {
        RCLCPP_INFO(get_logger(), "Setting target to %s", target.c_str());
        move_group->setNamedTarget(target);
    }

    void PlanAndMove()
    {
        RCLCPP_INFO(get_logger(), "Planning the motion");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto planing_result = move_group->plan(my_plan);

        if (planing_result != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to plan the motion");
        }

        RCLCPP_INFO(get_logger(), "Moving the robot");
        auto move_result = move_group->move();

        if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to move the robot");
        }
    }

    auto GetCurrentPose(const std::string &end_effector_link = "") const
    {
        return move_group->getCurrentPose(end_effector_link);
    }

    void SetTargetPose(const geometry_msgs::msg::Pose &target_pose)
    {
        moveit::core::RobotState target_robot_state(*(move_group->getCurrentState()));
        const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup(planning_group_);

        auto ik_result = target_robot_state.setFromIK(joint_model_group, target_pose);
        RCLCPP_INFO_STREAM(get_logger(), "ik_result: " << ik_result);
        if (!ik_result) {
            throw std::runtime_error("Failed to calculate IK solution");
        }

        std::vector<double> joint_group_positions;
        target_robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);

        RCLCPP_INFO_STREAM(get_logger(), "Setting target pose to: ");
        for (size_t i = 0; i < joint_group_positions.size(); ++i) {
            RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << joint_group_positions[i]);
        }

        move_group->setJointValueTarget(joint_group_positions);
    }

    std::string GetPlanningGroup() const
    {
        return planning_group_;
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::PoseStamped &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sheader: {frame_id: %s, stamp: %d.%d}\npose: {position: [%lf, %lf, %lf], orientation: [%lf, %lf, %lf, %lf]}",
                    prefix.c_str(), msg.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::TransformStamped &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sheader: {frame_id: %s, stamp: %d.%d}\ntransform: {translation: [%lf, %lf, %lf], rotation: [%lf, %lf, %lf, %lf]}",
                    prefix.c_str(), msg.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z,
                    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::Pose &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sposition: [%lf, %lf, %lf], orientation: [%lf, %lf, %lf, %lf]",
                    prefix.c_str(),
                    msg.position.x, msg.position.y, msg.position.z,
                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    }

private:
    std::string planning_group_;
};

class MoveGraspService : public rclcpp::Node
{
public:
    MoveGraspService(const std::string &node_name, std::shared_ptr<MoveItNode> moveit_node)
        : Node(node_name), moveit_node_(std::move(moveit_node))
    {
        move_to_service_               = this->create_service<move_grasp_msg::srv::MoveTo>("move_to", std::bind(&MoveGraspService::MoveToCallback, this, std::placeholders::_1, std::placeholders::_2));
        get_end_effector_pose_service_ = this->create_service<move_grasp_msg::srv::GetEndEffectorPose>("get_end_effector_pose", std::bind(&MoveGraspService::GetEndEffectorPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    std::shared_ptr<MoveItNode> moveit_node_;
    rclcpp::Service<move_grasp_msg::srv::MoveTo>::SharedPtr move_to_service_;
    rclcpp::Service<move_grasp_msg::srv::GetEndEffectorPose>::SharedPtr get_end_effector_pose_service_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void GetEndEffectorPoseCallback(const std::shared_ptr<move_grasp_msg::srv::GetEndEffectorPose::Request> request, std::shared_ptr<move_grasp_msg::srv::GetEndEffectorPose::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Received get_end_effector_pose request");
        (void)request; // Unused
        response->pose_stamped = moveit_node_->GetCurrentPose();
        response->frame_name   = moveit_node_->move_group->getEndEffectorLink();
    }

    void MoveToCallback(const std::shared_ptr<move_grasp_msg::srv::MoveTo::Request> request, std::shared_ptr<move_grasp_msg::srv::MoveTo::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Received move_to request:");
        auto frame_name = request->pose_stamped.header.frame_id.c_str();
        RCLCPP_INFO(get_logger(), "    frame_id: %s", frame_name);
        MoveItNode::Print(get_logger(), request->pose_stamped.pose, "    ");

        auto planning_frame_name = moveit_node_->move_group->getPlanningFrame().c_str();
        RCLCPP_INFO(get_logger(), "Planning frame: %s", planning_frame_name);

        // Get the transform from the request frame to the planning frame
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(planning_frame_name, frame_name, tf2::TimePointZero);
        } catch (const tf2::TransformException &e) {
            response->is_success = false;
            response->message    = e.what();
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", frame_name, planning_frame_name, e.what());
            return;
        }

        // Transform the request frame to the planning frame
        geometry_msgs::msg::Pose target_pose;
        tf2::doTransform(request->pose_stamped.pose, target_pose, t);
        MoveItNode::Print(get_logger(), target_pose, "Transformed pose: ");

        try {
            moveit_node_->SetTargetPose(target_pose);
        } catch (const std::exception &e) {
            response->is_success = false;
            response->message    = e.what();
            return;
        }

        try {
            moveit_node_->PlanAndMove();
        } catch (const std::exception &e) {
            response->is_success = false;
            response->message    = e.what();
            return;
        }

        response->is_success = true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const LOGGER = rclcpp::get_logger("move_grasp");

    auto const moveit_node = std::make_shared<MoveItNode>("move_grasp");
    moveit_node->Init();

    auto const service_node = std::make_shared<MoveGraspService>("move_grasp_service", moveit_node);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit_node);
    auto executor_thread = std::thread([&executor]() { executor.spin(); });

    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", moveit_node->move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", moveit_node->move_group->getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    std::stringstream ss;
    ss << "Available Planning Groups: ";
    std::copy(moveit_node->move_group->getJointModelGroupNames().begin(), moveit_node->move_group->getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(ss, ", "));
    RCLCPP_INFO_STREAM(LOGGER, ss.str());

    rclcpp::spin(service_node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}