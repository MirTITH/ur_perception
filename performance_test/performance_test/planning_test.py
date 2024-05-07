import rclpy
from rclpy.node import Node
from move_grasp_msg.srv import (
    GetEndEffectorPose,
    Grasp,
    MoveTo,
    MoveToNamed,
    Plan,
    GetJointPosition,
    MoveToJointPosition,
)
from geometry_msgs.msg import Pose
import time
import json
from typing import List
import os


class PlanningTest(Node):

    def __init__(self):
        super().__init__("planning_test")
        self.plan_client = self.create_client_wait(Plan, "move_grasp/plan")
        self.get_ee_pose_client = self.create_client_wait(GetEndEffectorPose, "move_grasp/get_end_effector_pose")
        self.get_joint_position_client = self.create_client_wait(GetJointPosition, "move_grasp/get_joint_position")
        self.move_to_client = self.create_client_wait(MoveTo, "move_grasp/move_to")
        self.move_to_joint_pos_client = self.create_client_wait(
            MoveToJointPosition, "move_grasp/move_to_joint_position"
        )

    def create_client_wait(self, srv_type, srv_name: str):
        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"service {client.srv_name} not available, waiting again...")
        return client

    def get_current_pose(self):
        request = GetEndEffectorPose.Request()
        future = self.get_ee_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result: GetEndEffectorPose.Response = future.result()
        if result is not None:
            return result.pose_stamped.pose
        else:
            self.get_logger().info(f"Service call failed: {future.exception()}")
            return None

    def get_joint_position(self):
        request = GetJointPosition.Request()
        future = self.get_joint_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result: GetJointPosition.Response = future.result()
        if result is not None:
            return result.joint_position.tolist()
        else:
            self.get_logger().info(f"Service call failed: {future.exception()}")
            return None

    # def plan_pose(self):
    #     request = Plan.Request()
    #     request.is_pose = True
    #     request.start_pose_stamped.pose.position.x = 0.0
    #     request.start_pose_stamped.pose.position.y = 0.0
    #     request.start_pose_stamped.pose.position.z = 0.0
    #     request.start_pose_stamped.pose.orientation.x = 0.0
    #     request.start_pose_stamped.pose.orientation.y = 0.0
    #     request.start_pose_stamped.pose.orientation.z = 0.0
    #     request.start_pose_stamped.pose.orientation.w = 1.0
    #     future = self.plan_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         self.get_logger().info(f"Plan successful: {future.result().success}")
    #     else:
    #         self.get_logger().info(f"Service call failed: {future.exception()}")

    def plan_joint_position(
        self,
        start_joint_position: List[float],
        target_joint_position: List[float],
        planner_id: str = "",
        planning_time: float = 1.0,
    ):
        request = Plan.Request()
        request.is_pose = False
        request.planner_id = planner_id
        request.planning_time = planning_time
        request.start_joint_position = start_joint_position
        request.target_joint_position = target_joint_position
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result: Plan.Response = future.result()
        if result is not None:
            # self.get_logger().info(f"is_success: {result.is_success}, message: {result.message}")
            return result
        else:
            self.get_logger().info(f"Service call failed: {future.exception()}")
            return None

    def move_to_pose(self, pose: Pose):
        request = MoveTo.Request()
        request.pose_stamped.pose = pose
        request.pose_stamped.header.frame_id = "world"
        future = self.move_to_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result: MoveTo.Response = future.result()
        if result is not None:
            return result.is_success, result.message
        else:
            self.get_logger().info(f"Service call failed: {future.exception()}")
            return False, ""

    def move_to_joint_position(self, joint_position: List[float]):
        request = MoveToJointPosition.Request()
        request.joint_position = joint_position
        future = self.move_to_joint_pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result: MoveToJointPosition.Response = future.result()
        if result is not None:
            return result.is_success, result.message
        else:
            self.get_logger().info(f"Service call failed: {future.exception()}")
            return False, ""


def dump_pose_list(pose_list: List[Pose], file_path: str):
    if not file_path.endswith(".jsonl"):
        file_path += ".jsonl"

    with open(file_path, "w") as file:
        for pose in pose_list:
            pose_vec = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            json.dump(pose_vec, file)
            file.write("\n")


def dump_joint_position_list(joint_position_list: List[List[float]], file_path: str):
    if not file_path.endswith(".json"):
        file_path += ".json"

    with open(file_path, "w") as file:
        json.dump(joint_position_list, file)
        file.write("\n")


def load_joint_position_list(file_path: str) -> List[List[float]]:
    if not file_path.endswith(".json"):
        file_path += ".json"
    with open(file_path, "r") as file:
        joint_position_list = json.load(file)
        return joint_position_list


def load_pose_list(file_path: str) -> List[Pose]:
    if not file_path.endswith(".jsonl"):
        file_path += ".jsonl"

    pose_list = []
    with open(file_path, "r") as file:
        for line in file:
            pose_vec = json.loads(line)
            pose = Pose()
            pose.position.x = pose_vec[0]
            pose.position.y = pose_vec[1]
            pose.position.z = pose_vec[2]
            pose.orientation.x = pose_vec[3]
            pose.orientation.y = pose_vec[4]
            pose.orientation.z = pose_vec[5]
            pose.orientation.w = pose_vec[6]

            pose_list.append(pose)
    return pose_list


def record_poses(planning_test: PlanningTest):
    joint_pos_list = []

    while rclpy.ok():
        cha = input("Press Enter to record pose, y to save poses and exit: ")
        if cha == "y":
            break
        joint_pos = planning_test.get_joint_position()
        if joint_pos is not None:
            print(f"Current pose: {joint_pos}")
            joint_pos_list.append(joint_pos)

    file_path = input("Enter file path to save joint positions: ")
    dump_joint_position_list(joint_pos_list, file_path)

    test_load_list = load_joint_position_list(file_path)
    print(f"Loaded joint positions: {test_load_list}")


def inspect_environment_pose(planning_test: PlanningTest):
    file_path = input("Enter file path to load poses: ")
    if file_path == "":
        file_path = "inspect_environment.jsonl"
    poses = load_pose_list(file_path)

    for i, pose in enumerate(poses):
        print(f"Moving to pose {i + 1}/{len(poses)}:")
        retry_count = 0
        while retry_count < 3:
            is_success, msg = planning_test.move_to_pose(pose)
            if is_success:
                print(f"Success.")
                break
            else:
                retry_count += 1
                print(f"    Failed: {msg} Retrying...")

        if retry_count == 3:
            print(f"Failed to move to pose {i + 1}/{len(poses)}: {msg}")


def inspect_environment_joint_pos(planning_test: PlanningTest):
    file_path = input("Enter file path to load joint positions: ")
    if file_path == "":
        file_path = "inspect_environment.json"
    joint_positions = load_joint_position_list(file_path)

    for i, joint_pos in enumerate(joint_positions):
        print(f"Moving to pose {i + 1}/{len(joint_positions)}:")
        retry_count = 0
        while retry_count < 3:
            is_success, msg = planning_test.move_to_joint_position(joint_pos)
            if is_success:
                print(f"Success.")
                break
            else:
                retry_count += 1
                print(f"    Failed: {msg} Retrying...")

        if retry_count == 3:
            print(f"Failed to move to pose {i + 1}/{len(joint_positions)}: {msg}")


def make_single_statistic_entry(is_success: bool, ik_time_ms: float, distance: float, planning_time_ms: float):
    return {
        "is_success": is_success,
        "ik_time_ms": ik_time_ms,
        "distance": distance,
        "planning_time_ms": planning_time_ms,
    }


def test_planning_joint_pos(
    planning_test: PlanningTest,
    joint_positions: List[List[float]],
    planner_id: str = "geometric::RRTConnect",
    planning_time: float = 1.0,
):
    stats = []
    for i, start_pos in enumerate(joint_positions):
        for j, target_pos in enumerate(joint_positions):
            if i == j:
                continue

            print(f"Planning from pose {i + 1}/{len(joint_positions)} to pose {j + 1}/{len(joint_positions)}:")
            stat = []
            for count in range(5):
                print(f"    Attempt {count + 1}/5:")
                result = planning_test.plan_joint_position(start_pos, target_pos, planner_id, planning_time)
                if result is not None:
                    print(f"    is_success: {result.is_success}, message: {result.message}")
                    ik_time_ms = result.ik_time_ms
                    distance = result.distance
                    planning_time_ms = result.planning_time_ms
                    print(f"    IK time: {ik_time_ms} ms, distance: {distance}, planning time: {planning_time_ms} ms")
                    stat.append(make_single_statistic_entry(result.is_success, ik_time_ms, distance, planning_time_ms))
                else:
                    print(
                        f"    Failed to plan from pose {i + 1}/{len(joint_positions)} to pose {j + 1}/{len(joint_positions)}"
                    )
                    stat.append(make_single_statistic_entry(False, -1, -1, -1))

            stats.append(
                {
                    "from": i,
                    "to": j,
                    "stat": stat,
                }
            )

    return stats


def test_and_save_planning_joint_pos(
    planning_test: PlanningTest,
    position_file_path: str,
    planner_id: str = "geometric::RRTConnect",
    planning_time: float = 1.0,
    save_folder: str = "benchmark_result",
):
    joint_positions = load_joint_position_list(position_file_path)
    # joint_positions = joint_positions[:3]
    stats = test_planning_joint_pos(planning_test, joint_positions, planner_id, planning_time)
    save_path = os.path.join(save_folder, f"{os.path.basename(position_file_path)}_{planner_id}_{planning_time}s.json")
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    with open(save_path, "w") as file:
        json.dump(stats, file)


def main():
    print("Hello, planning test!")

    rclpy.init()

    planning_test = PlanningTest()

    # record_poses(planning_test)

    # inspect_environment_joint_pos(planning_test)

    save_folder = "benchmark_result/no_env"

    # RRTkConfigDefault
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )

    # RRTConnectkConfigDefault
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTConnectkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTConnectkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )

    # RRTstarkConfigDefault
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=2.0,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=2.0,
        save_folder=save_folder,
    )

    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=1.0,
        save_folder=save_folder,
    )

    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.5,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.5,
        save_folder=save_folder,
    )

    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.2,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.2,
        save_folder=save_folder,
    )

    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="self_collision_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.1,
        save_folder=save_folder,
    )
    test_and_save_planning_joint_pos(
        planning_test=planning_test,
        position_file_path="environment_pos.json",
        planner_id="RRTstarkConfigDefault",
        planning_time=0.1,
        save_folder=save_folder,
    )


if __name__ == "__main__":
    main()
