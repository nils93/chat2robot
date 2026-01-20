from langchain_core.tools import tool
from pydantic import BaseModel, Field

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray

import math
import threading
import time
from threading import Event

from typing import TypedDict, List
from langchain_core.messages import BaseMessage


# ==========================================================
# LangGraph State
# ==========================================================

class AgentState(TypedDict):
    messages: List[BaseMessage]


# ==========================================================
# Global ROS + WAIT Event
# ==========================================================

ros_object = None
WAIT_EVENT = Event()


# ==========================================================
# Goal Input
# ==========================================================

class GoalInput(BaseModel):
    x: float = Field(description="X position in map frame")
    y: float = Field(description="Y position in map frame")
    theta: float = Field(description="Yaw angle in radians")


# ==========================================================
# ROS Node
# ==========================================================

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

        self.nav_status_code = 0  # STATUS_UNKNOWN

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_callback,
            10
        )

    def publish_goal(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)

        self.publisher.publish(msg)
        self.get_logger().info(f"Goal published: x={x}, y={y}, theta={theta}")

    def _status_callback(self, msg_status):
        if not msg_status.status_list:
            return
        self.nav_status_code = msg_status.status_list[-1].status

    def get_status_code(self):
        return self.nav_status_code


# ==========================================================
# ROS Init Helper
# ==========================================================

def init_ros_if_needed():
    global ros_object

    if ros_object is None:
        rclpy.init()
        ros_object = ROSGoalPublisher()
        threading.Thread(
            target=rclpy.spin,
            args=(ros_object,),
            daemon=True
        ).start()


# ==========================================================
# Navigation Status Monitor (KEY PART)
# ==========================================================

def monitor_navigation_status():
    """
    Wartet im Hintergrund, bis STATUS_SUCCEEDED erreicht ist,
    und setzt dann das WAIT_EVENT.
    """
    while not WAIT_EVENT.is_set():
        status = ros_object.get_status_code()

        if status == 4:  # STATUS_SUCCEEDED
            WAIT_EVENT.set()
            break

        time.sleep(0.3)


def start_navigation_monitor():
    WAIT_EVENT.clear()
    threading.Thread(
        target=monitor_navigation_status,
        daemon=True
    ).start()


# ==========================================================
# LangChain Tools
# ==========================================================

@tool("ROS_send_first_goal", args_schema=GoalInput)
def ROS_send_first_goal(x: float, y: float, theta: float) -> str:
    """
    Sendet das erste Navigationsziel an ROS2.
    """
    init_ros_if_needed()
    start_navigation_monitor()
    ros_object.publish_goal(x, y, theta)
    return f"Ziel 1 gesendet: x={x}, y={y}, theta={theta}"


@tool("ROS_send_second_goal", args_schema=GoalInput)
def ROS_send_second_goal(x: float, y: float, theta: float) -> str:
    """
    Sendet das zweite Navigationsziel an ROS2.
    """
    init_ros_if_needed()
    start_navigation_monitor()
    ros_object.publish_goal(x, y, theta)
    return f"Ziel 2 gesendet: x={x}, y={y}, theta={theta}"


@tool
def ROS_get_navigation_status() -> str:
    """
    Gibt den aktuellen Navigationsstatus zurück.
    """
    init_ros_if_needed()
    return str(ros_object.get_status_code())


# ==========================================================
# LangGraph WAIT State (BLOCKIERT)
# ==========================================================

@tool
def wait_state(state: AgentState) -> AgentState:
    """
    Blockiert den Agenten, bis TurtleBot3 die Zielpose erreicht hat.
    """
    WAIT_EVENT.wait()
    return state


# ==========================================================
# Beispiel Rechentool
# ==========================================================

@tool
def rechner(a: int, b: int, operation: str) -> int:
    if operation == "add":
        return a + b
    elif operation == "subtract":
        return a - b
    elif operation == "multiply":
        return a * b
    elif operation == "divide":
        return a / b if b != 0 else 0
    else:
        raise ValueError("Unbekannte Operation")
