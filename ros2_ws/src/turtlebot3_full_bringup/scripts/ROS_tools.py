from langchain_core.tools import tool
from pydantic import BaseModel, Field

#ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import math
import threading


class GoalInput(BaseModel):
    x: float = Field(description="X position in map frame")
    y: float = Field(description="Y position in map frame")
    theta: float = Field(description="Yaw angle in radians")
class ROSGoalPublisher(Node):
    def __init__(self):
        super().__init__('llm_goal_publisher')
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

ros_node = None
@tool("ROS_send_goal", args_schema=GoalInput)
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Sendet ein Navigationsziel an ROS2.
    """
    global ros_node

    if ros_node is None:
        rclpy.init()
        ros_node = ROSGoalPublisher()
        threading.Thread(
            target=rclpy.spin,
            args=(ros_node,),
            daemon=True
        ).start()

    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = ros_node.get_clock().now().to_msg()

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0

    msg.pose.orientation.z = math.sin(theta / 2.0)
    msg.pose.orientation.w = math.cos(theta / 2.0)

    ros_node.publisher.publish(msg)
    ros_node.get_logger().info(f"Goal published: x={x}, y={y}, theha={theta}")

    return f"Ziel gesendet: x={x}, y={y}, theta={theta}"

