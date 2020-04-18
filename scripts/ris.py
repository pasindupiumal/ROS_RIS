#! /usr/bin/env python

import math

# import ros libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from tf.transformations import euler_from_quaternion

#declare global variables
velocity_publisher_ = None
laser_subscriber_ = None
odom_subscriber_ = None
forward_speed_ = 0.18
right_wall_distance_ = 0
current_distance_ = 0
state_ = 0
t0_ = None
t1_ = None
move_count_ = 0
stop_count_ = 0
initial_stage_ = True
roll_ = pitch_ = yaw_ = 0.0
target_angle_ = 90
kP_ = 0.5
is_rotating_ = False
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
    2: 'Rotating',
}


#Subscriber callback methods

def change_state(state):

    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s', state, state_dict_[state])
        state_ = state


def laserCallback(msg):

    global regions_, right_wall_distance_

    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    right_wall_distance_ = round(regions_['right'], 2)

    rospy.loginfo('Distance from wall - %f', right_wall_distance_)


def odomCallback(msg):

    global roll_, pitch_, yaw_

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_, pitch_, yaw_) = euler_from_quaternion (orientation_list)


#Turtlebot action methods

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


def rotate():

    global target_angle_, velocity_publisher_, yaw_, kP_, is_rotating_

    msg = Twist()

    is_rotating_ = True

    #Calculate the radius value
    target_rad = target_angle_ * math.pi / 180

    target_rad = target_angle_ * math.pi/180

    if ( abs ( target_rad - yaw_) > 0.01 ):

        msg.angular.z = kP_  * (target_rad - yaw_)
    
    else:
        
        msg.angular.z = 0
        is_rotating_ = False

    velocity_publisher_.publish(msg)



def main():

    #Declare variables

    global velocity_publisher_, laser_subscriber_, initial_stage_, regions_, right_wall_distance_, odom_subscriber_
    front_wall_distance = 0.5
    laser_max = 10.0

    #Initialize ros node

    rospy.init_node('ris', anonymous=True)

    velocity_publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    laser_subscriber_ = rospy.Subscriber('/scan', LaserScan, laserCallback )

    odom_subscriber_ = rospy.Subscriber('/odom', Odometry, odomCallback)



    rate = rospy.Rate(10)

    #Run the node until user exists

    while not rospy.is_shutdown():

        msg = Twist()


        change_state(2)
        rotate()

        rate.sleep()


if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
