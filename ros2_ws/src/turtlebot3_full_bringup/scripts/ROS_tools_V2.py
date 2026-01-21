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
import time


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

        self.nav_status_code = 0  # =unknown

        self.create_subscription(  # sub für action status nav2
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_callback,
            10
        )
    
    def publish_goal(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = ros_object.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)

        self.publisher.publish(msg)
        self.get_logger().info(f"Goal published: x={x}, y={y}, theta={theta}")

    def _status_callback(self, msg_status):
        """
        (automatic) Callback: returns the navigation status for latest published goal
        """
        if len(msg_status.status_list) == 0:  # kein status vorhanden
            return

        self.nav_status_code = msg_status.status_list[-1].status

    def get_status_code(self):
        """
        returns status code of navigation
        """
        return self.nav_status_code
    
    def wait_for_goal_completion(self, timeout=60.0, check_interval=0.1):
        """
        Waits until the goal is either succeeded, canceled, or aborted.
        
        Args:
            timeout: Maximum time to wait in seconds (default: 60)
            check_interval: How often to check status in seconds (default: 0.1)
        
        Returns:
            tuple: (success: bool, status_code: int, message: str)
        """
        start_time = time.time()
        
        # Wait for goal to start executing (status changes from UNKNOWN/ACCEPTED)
        while time.time() - start_time < timeout:
            status = self.get_status_code()
            
            # STATUS_SUCCEEDED = 4
            if status == 4:
                return (True, status, "Ziel erfolgreich erreicht")
            
            # STATUS_CANCELED = 5
            elif status == 5:
                return (False, status, "Navigation abgebrochen")
            
            # STATUS_ABORTED = 6
            elif status == 6:
                return (False, status, "Navigation fehlgeschlagen")
            
            time.sleep(check_interval)
            rclpy.spin_once(self, timeout_sec=0)  # Process callbacks
        
        return (False, self.get_status_code(), f"Timeout nach {timeout}s erreicht")


@tool("ROS_send_goal", args_schema=GoalInput)
def ROS_send_goal(x: float, y: float, theta: float) -> str:
    """
    Verwenden, wenn auf Posen gefahren werden soll.
    Übergabeparameter sind die X-Koordinate, Y-Koordinate und der Winkel Theta. 
    Sendet ein Navigationsziel an ROS2.
    """
    global ros_object

    if ros_object is None:
        return "FEHLER: ROS2 noch nicht initialisiert"
    
    ros_object.publish_goal(x, y, theta)
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
    success, status_code, message = ros_object.wait_for_goal_completion(timeout=120.0)
    
    if not success:
        return f"Erstes Ziel nicht erreicht: {message} (Status: {status_code})"
    
    # First goal succeeded, send second goal
    ros_object.publish_goal(x_2, y_2, theta_2)
    
    return (f"Erstes Ziel erreicht: x={x_1}, y={y_1}, theta={theta_1}. "
            f"Zweites Ziel gesendet: x={x_2}, y={y_2}, theta={theta_2}")


@tool
def ROS_get_navigation_status() -> str:
    """
    Rufe dieses Tool auf um zu prüfen ob der Roboter sein Ziel erreicht hat.
    Gibt den aktuellen Navigationsstatus zurück.

    Status Codes:
    0 = STATUS_UNKNOWN (Unbekannt)
    1 = STATUS_ACCEPTED (Akzeptiert)
    2 = STATUS_EXECUTING (In Ausführung)
    3 = STATUS_CANCELING (Wird abgebrochen)
    4 = STATUS_SUCCEEDED (Erfolgreich)
    5 = STATUS_CANCELED (Abgebrochen)
    6 = STATUS_ABORTED (Fehlgeschlagen)
    """
    global ros_object

    if ros_object is None:
        return "FEHLER: ROS2 noch nicht initialisiert"

    status_code = ros_object.get_status_code()
    
    status_map = {
        0: "Unbekannt",
        1: "Akzeptiert",
        2: "In Ausführung",
        3: "Wird abgebrochen",
        4: "Erfolgreich",
        5: "Abgebrochen",
        6: "Fehlgeschlagen"
    }
    
    status_text = status_map.get(status_code, f"Unbekannter Status ({status_code})")
    
    return f"Status: {status_text} (Code: {status_code})"