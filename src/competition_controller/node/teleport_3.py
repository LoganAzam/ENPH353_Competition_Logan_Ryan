#! /usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Int32

def teleport_3():
    rospy.init_node('teleport_node')
    
    # 1. Define the Dirt Road Position [x, y, z, ox, oy, oz, ow]
    # These coordinates are based on your previous examples
    # Orientation is a Quaternion for 90-degree yaw (z=0.707, w=0.707)
    dirt_road_pos = [-3.97354, -2.32731, 0.05, 0.0, 0.0, -0.6991, 0.7150]
    msg = ModelState()
    msg.model_name = 'B1'
    msg.pose.position.x = dirt_road_pos[0]
    msg.pose.position.y = dirt_road_pos[1]
    msg.pose.position.z = dirt_road_pos[2]
    msg.pose.orientation.x = dirt_road_pos[3]
    msg.pose.orientation.y = dirt_road_pos[4]
    msg.pose.orientation.z = dirt_road_pos[5]
    msg.pose.orientation.w = dirt_road_pos[6]

    # 3. Call Gazebo Teleport Service
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(msg)
        rospy.loginfo("Robot teleported to Dirt Road.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    # 4. Update the Robot's Logic State
    # Tell the other nodes we are now in State 3 (Dirt Road)
    state_pub = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)
    rospy.sleep(0.5) # Give the publisher a moment to connect
    state_pub.publish(6)
    rospy.loginfo("State changed to 6 (Mountain).")

if __name__ == '__main__':
    try:
        teleport_3()
    except rospy.ROSInterruptException:
        pass