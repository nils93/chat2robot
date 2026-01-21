from langchain_core.tools import tool
from pydantic import BaseModel, Field
import time

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

class TwoGoalsInput(BaseModel):
    x_1: float = Field(description="X position of first goal in map frame")
    y_1: float = Field(description="Y position of first goal in map frame")
    theta_1: float = Field(description="Yaw angle of first goal in radians")
    x_2: float = Field(description="X position of second goal in map frame")
    y_2: float = Field(description="Y position of second goal in map frame")
    theta_2: float = Field(description="Yaw angle of second goal in radians")

ros_object = None

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

@tool("ROS_send_goal", args_schema=GoalInput)
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Verwenden, wenn auf Posen gefahren werden soll. \n
    Übergabeparameter sind die X-Koordinate, Y-Koordinate und der Winkel Theta. Sendet ein Navigationsziel an ROS2.
    """
    global ros_object

    if ros_object is None:
        return "FEHLER: ROS2 noch nicht initialisiert"
    
    ros_object.publish_goal(x,y,theta)
    return f"Ziel gesendet: x={x}, y={y}, theta={theta}"

@tool("ROS_send_two_goals", args_schema=TwoGoalsInput)
def ROS_send_two_goals(x_1: float, y_1: float, theta_1: float, 
                       x_2: float, y_2: float, theta_2: float) -> str:
    """
    Verwenden, wenn auf zwei Posen nacheinander gefahren werden soll.
    Übergabeparameter sind:
    x_1=X-Koordinate von Pose 1
    y_1=Y-Koordinate von Pose 1
    theta_1=Theta Winkel von Pose 1
    x_2=X-Koordinate von Pose 2
    y_2=Y-Koordinate von Pose 2
    theta_2=Theta Winkel von Pose 2
    
    Sendet zwei Navigationsziele nacheinander an ROS2. Wartet bis Ziel 1 erreicht ist, 
    bevor Ziel 2 gesendet wird.
    """
    global ros_object

    if ros_object is None:
        return "FEHLER: ROS2 noch nicht initialisiert"
    
    # Send first goal
    ros_object.publish_goal(x_1, y_1, theta_1)
    
    # Wait for first goal to complete
    #success, status_code, message = ros_object.wait_for_goal_completion(timeout=120.0)
    #while (anhgekommen != true)

    while ros_object.get_status_code()!=4:            #ros_object.goal_reached():
        time.sleep(0.2)
    
    #if not success:
    #    return f"Erstes Ziel nicht erreicht: {message} (Status: {status_code})"
    
    # First goal succeeded, send second goal
    ros_object.publish_goal(x_2, y_2, theta_2)
    
    return (f"Erstes Ziel erreicht: x={x_1}, y={y_1}, theta={theta_1}. "
            f"Zweites Ziel gesendet: x={x_2}, y={y_2}, theta={theta_2}")

@tool
def ROS_get_navigation_status():
    """
    Rufe dieses Tool auf um zu prüfen ob der Roboter sein Ziel erreicht hat.
    Gibt den aktuellen Navigationsstatus zurück. Entweder als Status-Code, oder als interpretierter kurzer Text.

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

    status_code = ros_object.get_status_code()
    return str(status_code)