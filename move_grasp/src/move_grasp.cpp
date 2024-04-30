#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

void Plan(moveit::planning_interface::MoveGroupInterface &move_group)
{
    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.move();
    } else {
        throw std::runtime_error("Failed to plan the motion");
    }
}

void SetTargetPose(moveit::planning_interface::MoveGroupInterface &move_group, const std::string &planning_group, const geometry_msgs::msg::Pose &target_pose)
{
    moveit::core::RobotState target_robot_state(*move_group.getCurrentState());
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
    target_robot_state.setFromIK(joint_model_group, target_pose);
    std::vector<double> joint_group_positions;
    target_robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setJointValueTarget(joint_group_positions);
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const move_group_node = std::make_shared<rclcpp::Node>(
        "move_grasp",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("move_grasp");

    static const std::string PLANNING_GROUP = "ur_manipulator";

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Create the Planning Scene Interface to add and remove objects from the world
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    // Planning frame: world

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    // End effector link: tool0

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    move_group.setNamedTarget("home");
    try {
        Plan(move_group);
    } catch (const std::exception &) {
        RCLCPP_ERROR(LOGGER, "Failed to move to home position");
    }

    auto ee_pose = move_group.getCurrentPose();

    RCLCPP_INFO(LOGGER, "End effector pos: [%lf, %lf, %lf]",
                ee_pose.pose.position.x,
                ee_pose.pose.position.y,
                ee_pose.pose.position.z);

    // Set the target pose for the end effector
    geometry_msgs::msg::Pose target_pose = ee_pose.pose;
    target_pose.position.z += 0.1;

    SetTargetPose(move_group, PLANNING_GROUP, target_pose);

    try {
        Plan(move_group);
    } catch (const std::exception &) {
        RCLCPP_ERROR(LOGGER, "Failed to move to target_pose");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}