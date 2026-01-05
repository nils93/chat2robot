#hier den Publisher implementieren


#->Hier auf GoalPose publishen, ROS2 macht dann den rest!
#Objekt mit Methode schreiben, die die Koordinaten als übergabeparameter erhält und dann published

#Format:


# ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
#   header: {
#     frame_id: 'map'
#   },
#   pose: {
#     position: {x: 1.0, y: 0.5, z: 0.0},
#     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#   }
# }"


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def publish_goal_pose():
    rclpy.init()
    node = Node('goal_pose_publisher')
    
    # Publisher für das goal_pose Topic erstellen
    publisher = node.create_publisher(PoseStamped, 'goal_pose', 10)
    
    # Kurz warten, damit der Publisher sich registrieren kann
    rclpy.spin_once(node, timeout_sec=0.5)
    
    # PoseStamped Nachricht erstellen
    goal_msg = PoseStamped()
    
    # Header setzen
    goal_msg.header.stamp = node.get_clock().now().to_msg()
    goal_msg.header.frame_id = 'map'  # oder 'odom' je nach deinem Setup
    
    # Position nahe bei 0:0:0 (z.B. 1 Meter in x-Richtung)
    goal_msg.pose.position.x = 1.0
    goal_msg.pose.position.y = 0.5
    goal_msg.pose.position.z = 0.0
    
    # Orientation (Quaternion für keine Rotation um z-Achse)
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0
    
    # Nachricht publishen
    publisher.publish(goal_msg)
    node.get_logger().info(f'Goal pose published: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}')
    
    # Kurz warten, damit die Nachricht gesendet wird
    rclpy.spin_once(node, timeout_sec=0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_goal_pose()