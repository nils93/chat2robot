from langchain_core.tools import tool
from pydantic import BaseModel, Field

#ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray

import math
import threading


class GoalInput(BaseModel):
    x: float = Field(description="X position in map frame")
    y: float = Field(description="Y position in map frame")
    theta: float = Field(description="Yaw angle in radians")

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

        self.nav_status_code = 0 #=unknown

        self.create_subscription( #sub für action status nav2
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_callback,
            10
        )
        
    
    def publish_goal(self,x,y,theta):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = ros_object.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)

        self.publisher.publish(msg)
        self.get_logger().info(f"Goal published: x={x}, y={y}, theha={theta}")

    def _status_callback(self, msg_status):
        """
        (automatic) Callback: returns the navigation status for latest published goal"
        """

        if len(msg_status.status_list) == 0: #kein status vorhanden
            return

        self.nav_status_code = msg_status.status_list[-1].status

    def get_status_code(self):
        """
        returns status code of navigation
        """
        return self.nav_status_code
    
ros_object = None

@tool("ROS_send_goal", args_schema=GoalInput)
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Verwenden, wenn auf eine Pose gefahren werden soll. Übergabeparameter sind die X-Koordinate, Y-Koordinate und der WInkel Theta. Sendet ein Navigationsziel an ROS2.
    """
    global ros_object

    if ros_object is None:
        rclpy.init()
        ros_object = ROSGoalPublisher()
        threading.Thread(
            target=rclpy.spin,
            args=(ros_object,),
            daemon=True
        ).start()

    ros_object.publish_goal(x,y,theta)
    return f"Ziel gesendet: x={x}, y={y}, theta={theta}"

@tool
def ROS_get_navigation_status():
    """
    Returns current naviagtion status of the last navigation goal. Interprets the status_code
    From ROS2-documentation:

    Compact Message Definition
    int8 STATUS_UNKNOWN=0
    int8 STATUS_ACCEPTED=1
    int8 STATUS_EXECUTING=2
    int8 STATUS_CANCELING=3
    int8 STATUS_SUCCEEDED=4
    int8 STATUS_CANCELED=5
    int8 STATUS_ABORTED=6
    action_msgs/msg/GoalInfo goal_info
    int8 status
    """

    global ros_object

    if ros_object is None:
        rclpy.init()
        ros_object = ROSGoalPublisher()
        threading.Thread(
            target=rclpy.spin,
            args=(ros_object,),
            daemon=True
        ).start()
        

    status_code = ros_object.get_status_code()
    return str(status_code)


@tool
def rechner(a: int, b: int, operation: str) -> int:
    """Führt eine mathematische Operation durch.
    
    Args:
        a: Erste Zahl
        b: Zweite Zahl
        operation: Die Operation ('add', 'subtract', 'multiply', 'divide')
    """
    if operation == "add":
        return a + b
    elif operation == "subtract":
        return a - b
    elif operation == "multiply":
        return a * b
    elif operation == "divide":
        return a / b if b != 0 else "Division durch Null!"
    else:
        return "Unbekannte Operation"

