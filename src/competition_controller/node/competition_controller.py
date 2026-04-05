#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32 

def master_controller():
    # 1. Initialize the node ONCE with a single name
    rospy.init_node('master_controller')

    # 2. Define all your publishers
    # This one talks to the simulation scoring system
    pub_score = rospy.Publisher('/score_tracker', String, queue_size=1) 
    # This one talks to your other nodes (like road_PID)
    pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

    # 3. The 1-second Rule
    # The manual requires a 1s delay to let ROS Master register the topics 
    rospy.sleep(1)

    # 4. Start the Competition Timer
    # Format: 'TeamName,password,0,NA'
    # This registers your team and starts the 4-minute clock
    start_msg = "TeamID,Password,0,NA" 
    pub_score.publish(start_msg) 
    rospy.loginfo("Timer started for team: TeamID")

    # 5. Set Initial State
    # 1 - road_PID
    pub_state.publish(1)
    rospy.loginfo("State changed to: road_PID")

    # Keep the node alive if you plan to add more logic later
    rospy.spin()

if __name__ == '__main__':
    try:
        master_controller()
    except rospy.ROSInterruptException:
        pass