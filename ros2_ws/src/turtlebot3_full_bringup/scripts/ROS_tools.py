from langchain_core.tools import tool
from pydantic import BaseModel, Field

# ROS2 / Nav2 Action
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import math


class GoalInput(BaseModel):
    x: float = Field(description="X position in map frame")
    y: float = Field(description="Y position in map frame")
    theta: float = Field(description="Yaw angle in radians")


ros_object = None


class ROSGoalPublisher(Node):
    def __init__(self):
        super().__init__("llm_goal_publisher")

        # Action client for Nav2 NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # navigation status code (your mapping)
        # 0 UNKNOWN, 1 ACCEPTED, 2 EXECUTING, 3 CANCELING, 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        self.nav_status_code = 0

    def publish_goal(self, x: float, y: float, theta: float):
        # Reset status for new goal
        self.nav_status_code = 0

        # Wait for Nav2 action server
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server '/navigate_to_pose' not available!")
            self.nav_status_code = 6  # ABORTED-like
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal async
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(f"Goal sent via action: x={x}, y={y}, theta={theta}")

    def _goal_response_callback(self, future):
        """Called once Nav2 answers whether the goal is accepted/rejected."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal response callback error: {e}")
            self.nav_status_code = 6
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.nav_status_code = 6
            return

        self.get_logger().info("Goal accepted by Nav2")
        self.nav_status_code = 2  # EXECUTING (good enough for UI)

        # Request final result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Called once Nav2 returns final result of navigation."""
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Result callback error: {e}")
            self.nav_status_code = 6
            return

        # result.status is an action status code:
        # SUCCEEDED=4, CANCELED=5, ABORTED=6  (matches your mapping)
        self.nav_status_code = int(result.status)

        if self.nav_status_code == 4:
            self.get_logger().info("Navigation SUCCEEDED (result)")
        elif self.nav_status_code == 5:
            self.get_logger().warn("Navigation CANCELED (result)")
        elif self.nav_status_code == 6:
            self.get_logger().error("Navigation ABORTED (result)")
        else:
            self.get_logger().info(f"Navigation finished with status={self.nav_status_code}")

    def get_status_code(self) -> int:
        return int(self.nav_status_code)


@tool("ROS_send_goal", args_schema=GoalInput)
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Verwenden, wenn auf Posen gefahren werden soll.
    Übergabeparameter sind die X-Koordinate, Y-Koordinate und der Winkel Theta.
    Sendet ein Navigationsziel an ROS2 (Nav2 Action: /navigate_to_pose).
    """
    global ros_object
    if ros_object is None:
        return "FEHLER: ROS2 noch nicht initialisiert"

    ros_object.publish_goal(x, y, theta)
    return f"Ziel gesendet: x={x}, y={y}, theta={theta}"


@tool
def ROS_get_navigation_status():
    """
    Prüft, ob der Roboter sein Ziel erreicht hat und gibt den Status-Code zurück.

    STATUS_UNKNOWN=0
    STATUS_ACCEPTED=1
    STATUS_EXECUTING=2
    STATUS_CANCELING=3
    STATUS_SUCCEEDED=4
    STATUS_CANCELED=5
    STATUS_ABORTED=6
    """
    global ros_object
    if ros_object is None:
        return "0"
    return str(ros_object.get_status_code())
