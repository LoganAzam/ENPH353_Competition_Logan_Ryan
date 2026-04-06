#! /usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Int32

locations = [
    [5.5001163, 2.1308477, 0.0400041, -0.0000099, 0.0000078, -0.7068494, 0.7073641],
    [5.5655525, -0.9639541, 0.0400135, 0.0000314, -0.0000231, -0.7657944, 0.6430854],
    [4.3958341, -2.1446789, 0.0400008, -0.0000046, -0.0000010, -0.7275628, -0.6860411],
    [0.4422774, -0.9636061, 0.0400205, 0.0000741, 0.0000047, -0.6504517, -0.7595476],
    [0.6586153, 1.8392030, 0.0400365, -0.0000820, -0.0000555, -0.6377086, 0.7702777],
    [-3.0799216, 1.5015467, 0.0400521, -0.0000115, 0.0000226, -0.9996809, -0.0252625],
    [-4.1672659, -1.7458726, 0.0400001, 0.0000002, 0.0000001, -0.2366042, 0.9716061],
    [-1.3679110, -1.2373768, 1.8543029, -0.0110468, -0.0151554, -0.2039262, -0.9788066]
]

currentIndex = 0
set_state = None
pub_state = None

def teleport_callback(event):
    global currentIndex
    global locations
    global set_state, pub_state

    if currentIndex >= len(locations):
        rospy.loginfo("Finished teleporting all locations.")
        return
    
    locationArray = locations[currentIndex]
    location = ModelState()
    location.model_name = 'B1'
    location.pose.position.x = locationArray[0]
    location.pose.position.y = locationArray[1]
    location.pose.position.z = locationArray[2]
    location.pose.orientation.x = locationArray[3]
    location.pose.orientation.y = locationArray[4]
    location.pose.orientation.z = locationArray[5]
    location.pose.orientation.w = locationArray[6]

    try:
        set_state(location)
        rospy.loginfo(f"Teleported B1 to {locationArray[:3]}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    pub_state.publish(0)
    currentIndex += 1

def main():
    global set_state
    global pub_state

    rospy.init_node('clueboard_teleporter_node')
    pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

    rospy.sleep(1)
    pub_state.publish(1)

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rospy.Timer(rospy.Duration(1.0), teleport_callback)

    rospy.spin()
        

if __name__ == '__main__':
    main()