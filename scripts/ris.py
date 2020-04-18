#! /usr/bin/env python

import math

# import ros libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

#declare global variables
velocity_publisher_ = None
laser_subscriber_ = None
forward_speed_ = 0.18
current_distance_ = 0
state_ = 0
t0_ = None
t1_ = None
move_count_ = 0
stop_count_ = 0
initial_stage_ = True
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_dict_ = {
    0: 'Stopped moving',
    1: 'Moving forward',
}



def change_state(state):

    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s', state, state_dict_[state])
        state_ = state


def laserCallback(msg):

    global regions_

    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    rospy.loginfo('Distance from wall - %f', regions_['right'])

    
def move_forward():

    global velocity_publisher_

    vel_msg = Twist()

    vel_msg.linear.x = forward_speed_
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0  
    
    velocity_publisher_.publish(vel_msg)


def stop_moving():

    global velocity_publisher_
    msg = Twist()

    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0.0 
    
    velocity_publisher_.publish(msg)



def main():

    #Declare variables

    global velocity_publisher_, laser_subscriber_, initial_stage_, regions_
    front_wall_distance = 0.5
    laser_max = 10.0

    #Initialize ros node

    rospy.init_node('ris', anonymous=True)

    velocity_publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    laser_subscriber_ = rospy.Subscriber('/scan', LaserScan, laserCallback )

    rate = rospy.Rate(10)

    #Run the node until user exists

    while not rospy.is_shutdown():

        msg = Twist()

        if regions_['right'] == laser_max:

            change_state(0)
            stop_moving()

        elif regions_['right'] >  1.1:

            change_state(0)
            stop_moving()

        elif regions_['front'] < front_wall_distance:

            change_state(0)
            stop_moving()

        else:

            change_state(1)
            move_forward()

        rate.sleep()


if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
