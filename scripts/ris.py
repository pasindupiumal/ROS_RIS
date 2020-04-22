#! /usr/bin/env python

import math

# import ros libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion

#declare global variables
velocity_publisher_ = None
laser_subscriber_ = None
odom_subscriber_ = None

forward_speed_ = 0.18

current_distance_ = 0

state_ = 0
stage_ = 1

t0_ = None
t1_ = None

roll_ = pitch_ = yaw_ = 0.0

target_angle_ = 0
kP_ = 1.0
angular_velocity_ = 0
target_rad_ = target_angle_ * math.pi / 180
is_stopped = False
should_rotate_ = 0

regions_ = {
        'one': 0,
        'two': 0,
        'three': 0,
        'four': 0,
        'five': 0,
        'six': 0,
        'seven': 0,
        'eight': 0,
        'nine': 0,
        'ten': 0,
}
state_dict_ = {
    0: 'Stopped moving',
    1: 'Moving forward',
    2: "Moving right",
    3: "Moving left",
}
stage_dict_ = {
    1: 'Stage 1',
    2: 'Stage 2',
}



#Subscriber callback methods

def change_state(state):

    global state_, state_dict_
    if state is not state_:
        print ("Wall follower - {} - {}".format(state, state_dict_[state]))
        state_ = state


def change_stage(stage):

    global stage_, stage_dict_
    if stage is not stage_:
        print ("Initiated - {}".format(stage_dict_[stage]))
        state_ = state


def laserCallback(msg):

    global regions_, angular_velocity_, yaw_, should_rotate_

    regions_ = {
        'one':  min(min(msg.ranges[0:71]), 10),
        'two': min(min(msg.ranges[72:143]), 10),
        'three':  min(min(msg.ranges[144:215]), 10),
        'four':  min(min(msg.ranges[216:287]), 10),
        'five':   min(min(msg.ranges[288:359]), 10),
        'six':  min(min(msg.ranges[360:431]), 10),
        'seven': min(min(msg.ranges[432:503]), 10),
        'eight':  min(min(msg.ranges[504:575]), 10),
        'nine':  min(min(msg.ranges[576:647]), 10),
        'ten':   min(min(msg.ranges[648:719]), 10),
    }

    print("DW - {} | AV - {} | Yaw - {} | SR - {}".format(regions_['three'], angular_velocity_, yaw_, should_rotate_))


def odomCallback(msg):

    global roll_, pitch_, yaw_, angular_velocity_, should_rotate_

    angular_velocity_ = msg.twist.twist.angular.z
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_, pitch_, yaw_) = euler_from_quaternion (orientation_list)

    should_rotate_ = abs ( target_rad_ - yaw_ )


#Turtlebot action methods

def stop_moving():

    global velocity_publisher_, is_stopped, yaw_, kP_, target_rad_, angular_velocity_

    msg = Twist()

    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0

    if ( abs ( target_rad_ - yaw_) > 0.0001 ):

        msg.angular.z = kP_  * (target_rad_ - yaw_)
    
    else:

        msg.angular.z = 0.0

    velocity_publisher_.publish(msg)

    if  abs(angular_velocity_) < 0.0000001:

        change_state(2)


def move_right():

    global velocity_publisher_, forward_speed_

    msg = Twist()

    msg.linear.x = forward_speed_
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = -0.1

    velocity_publisher_.publish(msg)

def move_left():

    global velocity_publisher_, forward_speed_

    msg = Twist()

    msg.linear.x = forward_speed_
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0.1

    velocity_publisher_.publish(msg)


def move_forward():

    global velocity_publisher_, yaw_, kP_, target_rad_, forward_speed_

    msg = Twist()

    msg.linear.x = forward_speed_

    if ( abs ( target_rad_ - yaw_) > 0.0001 ):

        msg.angular.z = kP_  * (target_rad_ - yaw_)
    
    else:

        msg.angular.z = 0.0


    velocity_publisher_.publish(msg)



def go_to_door():

    global regions_, is_stopped

    if ( is_stopped == False ):

            if regions_['three'] == 10:

                is_stopped = True

            if regions_['one'] > 1.1:

                change_state(2)
                move_right()

            elif regions_['one'] < 1.0:

                change_state(3)
                move_left()
            
            else:

                change_state(1)
                move_forward()

    else:

        change_state(0)
        stop_moving()



def main():

    #Declare variables

    global velocity_publisher_, laser_subscriber_, regions_, odom_subscriber_, stage_


    #Initialize ros node

    rospy.init_node('ris', anonymous=True)

    velocity_publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    laser_subscriber_ = rospy.Subscriber('/scan', LaserScan, laserCallback)

    odom_subscriber_ = rospy.Subscriber('/odom', Odometry, odomCallback)

    rate = rospy.Rate(10)

    #Run the node until user exists

    while not rospy.is_shutdown():

        if ( stage_ == 1 ):

            go_to_door()

        elif ( stage_ == 2 ):

            print "Started stage two"


        rate.sleep()


if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
