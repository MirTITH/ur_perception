#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>

class MoveItNode : public rclcpp::Node
{
public:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

public:
    MoveItNode(const std::string &node_name, const std::string &planning_group)
        : Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),

          PLANNING_GROUP_(planning_group){};

    void Init()
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP_);
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
        const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

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

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::PoseStamped &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sheader: {frame_id: %s, stamp: %d.%d}\npose: {position: [%lf, %lf, %lf], orientation: [%lf, %lf, %lf, %lf]}",
                    prefix.c_str(), msg.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    }

private:
    const std::string PLANNING_GROUP_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("move_grasp");
    RCLCPP_INFO(LOGGER, "Starting MoveItNode");
    auto const moveit_node = std::make_shared<MoveItNode>("move_grasp", "ur_manipulator");
    moveit_node->Init();
    RCLCPP_INFO(LOGGER, "MoveItNode Initialized");

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", moveit_node->move_group->getPlanningFrame().c_str());
    // Planning frame: world

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", moveit_node->move_group->getEndEffectorLink().c_str());
    // End effector link: tool0

    // We can get a list of all the groups in the robot:
    std::stringstream ss;
    ss << "Available Planning Groups: ";
    std::copy(moveit_node->move_group->getJointModelGroupNames().begin(), moveit_node->move_group->getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(ss, ", "));
    RCLCPP_INFO_STREAM(LOGGER, ss.str());

    auto ee_pose = moveit_node->GetCurrentPose();
    moveit_node->Print(LOGGER, ee_pose, "Current End Effector Pose: ");

    // Set the target pose for the end effector
    geometry_msgs::msg::Pose target_pose = ee_pose.pose;
    target_pose.position.z += 0.1;

    moveit_node->SetTargetPose(target_pose);

    moveit_node->PlanAndMove();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}