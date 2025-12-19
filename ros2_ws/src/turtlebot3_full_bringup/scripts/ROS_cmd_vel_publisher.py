#hier den Publisher implementieren

# import rospy
# from geometry_msgs.msg import Twist

# def send_velocity(linear, angular):
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     msg = Twist()
#     msg.linear.x = linear
#     msg.angular.z = angular
#     pub.publish(msg)