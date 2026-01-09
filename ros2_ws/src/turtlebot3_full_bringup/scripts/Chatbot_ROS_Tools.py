# 1. ACTION CLIENT ERSTELLEN
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from langchain_core.tools import tool
from pydantic import BaseModel, Field

import asyncio


#ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
import threading

#action client
#nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

class ROSGoalPublisher(Node):
    def __init__(self):
        super().__init__('llm_goal_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos
        )

ros_node = None
@tool
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Verwenden, wenn auf eine Pose gefahren werden soll. Übergabeparameter sind die X-Koordinate, Y-Koordinate und der WInkel Theta. Sendet ein Navigationsziel an ROS2.
    
    Args:
    x: X-Koordinate des Roboters
    y: Y-Koordinate des Roboters
    theta: Yaw-Angle in radians    
    """
    global ros_node

    if ros_node is None:        #wenn Klasse noch nicht instanziert: lege hier das Objekt an (instanziere sie)
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
