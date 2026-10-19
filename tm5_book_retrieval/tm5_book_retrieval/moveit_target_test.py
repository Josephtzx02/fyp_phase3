import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy


class MoveItTargetTest(Node):

    def __init__(self):
        super().__init__('moveit_target_test')

        self.robot = MoveItPy(node_name="moveit_target_test")
        self.arm = self.robot.get_planning_component("tmr_arm")

        self.get_logger().info("MoveIt interface initialized")

    def plan_target(self, x_mm, y_mm, z_mm):

        pose = Pose()

        # TM5 MoveIt uses metres
        pose.position.x = x_mm / 1000.0
        pose.position.y = y_mm / 1000.0
        pose.position.z = z_mm / 1000.0

        # Initial neutral orientation.
        # We will determine the final gripper orientation later.
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()

        self.arm.set_goal_state(
            pose_stamped=None,
            pose_link="flange"
        )

        self.get_logger().info(
            f"Planning target: "
            f"x={x_mm:.1f} mm, "
            f"y={y_mm:.1f} mm, "
            f"z={z_mm:.1f} mm"
        )

        result = self.robot.plan()

        if result:
            self.get_logger().info("✅ MoveIt planning succeeded")
        else:
            self.get_logger().error("❌ MoveIt planning failed")


def main():

    rclpy.init()

    node = MoveItTargetTest()

    # Step 17 test GraspTarget
    node.plan_target(
        x_mm=20.0,
        y_mm=-15.0,
        z_mm=300.0
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
