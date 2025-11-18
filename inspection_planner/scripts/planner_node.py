#!/usr/bin/env python3
from header import *
from inspection_planner.planner_core import PlannerCore


class PlannerNode(Node):
    def __init__(self):
        super().__init__('inspection_planner_node')

        # --- Instantiate non-ROS planner core ---
        self.planner = PlannerCore()

        # --- Publishers / Subscribers ---
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'current_pose', self.pose_callback, 10)

        # --- Optional trigger service to run planning manually ---
        self.plan_srv = self.create_service(Trigger, 'plan_path', self.plan_service)

        self.current_pose = None
        self.get_logger().info("Planner Node initialized.")


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down planner node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
