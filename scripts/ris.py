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

total_distance_ = 0
previous_x_ = 0
previous_y_ = 0
is_first_run_ = True

forward_speed_ = 0.18

current_distance_ = 0
current_distance_travelled_ = 0
distance_to_travel_ = 0

state_ = 0
stage_ = 1

t0_ = None
t1_ = None

roll_ = pitch_ = yaw_ = 0.0

target_angle_ = 0
kP_ = 1.0
angular_velocity_ = 0
target_rad_ = target_angle_ * math.pi / 180
stage1_switch_ = True
detected_door_number_ = 0
distance_to_door_from_start_ = 0

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
    3: 'Stage 3',
    4: 'Stage 4',
    5: 'Stage 5',
}



#Subscriber callback methods

def change_state(state):

    global state_, state_dict_
    if state is not state_:
        #print ("Wall follower - {} - {}".format(state, state_dict_[state]))
        state_ = state


def change_stage(stage):

    global stage_, stage_dict_
    if stage is not stage_:
        print ("\nInitiated - {}\n".format(stage_dict_[stage]))
        stage_ = stage


def laserCallback(msg):

    global regions_, angular_velocity_, yaw_, total_distance_, current_distance_travelled_, distance_to_travel_, t0_, t1_, stage_

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


    print("DW - {} | AV - {} | Yaw - {} | CDT - {} | DTM - {} | T0 - {} | T1 - {}".format(regions_['one'], angular_velocity_, yaw_, current_distance_travelled_, distance_to_travel_, t0_, t1_))


def odomCallback(msg):

    global roll_, pitch_, yaw_, angular_velocity_

    angular_velocity_ = msg.twist.twist.angular.z
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_, pitch_, yaw_) = euler_from_quaternion (orientation_list)

    distance_calculation(msg)


#Turtlebot action methods

def distance_calculation(msg):

    global previous_x_, previous_y_, total_distance_, is_first_run_

    if is_first_run_ == True:

        previous_x_ = msg.pose.pose.position.x
        previous_y_ = msg.pose.pose.position.y

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    distance_increment = math.hypot(current_x - previous_x_, current_y - previous_y_)

    total_distance_ = total_distance_ + distance_increment

    previous_x_ = current_x
    previous_y_ = current_y
    
    is_first_run_ = False


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

    change_stage(2)


def move_right():

    global velocity_publisher_, forward_speed_

    ang_vel1 = -0.04

    msg = Twist()

    msg.linear.x = forward_speed_
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = ang_vel1

    while ang_vel1 < 0:

        ang_vel1 = ang_vel1 + 0.01

    velocity_publisher_.publish(msg)

def move_left():

    global velocity_publisher_, forward_speed_

    ang_vel = 0.1

    msg = Twist()

    msg.linear.x = forward_speed_
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = ang_vel


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


def move_forward_by_distance(distance_to_move, degree_to_maintain, next_stage):

    global forward_speed_, velocity_publisher_, t0_, t1_, current_distance_travelled_, distance_to_travel_

    distance_to_travel_ = distance_to_move

    angle_in_rad = degree_to_maintain * math.pi / 180

    msg = Twist()

    msg.linear.x = forward_speed_

    while ( current_distance_travelled_ < distance_to_move ):

        if ( abs ( angle_in_rad - yaw_) > 0.0001 ):

            msg.angular.z = kP_  * (angle_in_rad - yaw_)
    
        else:

            msg.angular.z = 0.0

        velocity_publisher_.publish(msg)

        t1_ = rospy.Time.now().to_sec()

        current_distance_travelled_ = forward_speed_ * ( t1_ - t0_)


    msg.linear.x = 0.0

    velocity_publisher_.publish(msg)

    change_stage(next_stage)


def rotate(angle):

    global velocity_publisher_, yaw_, kP_

    angle_in_rad = angle * math.pi / 180

    msg = Twist()


    while ( abs ( angle_in_rad - yaw_) > 0.001 ):

        msg.angular.z = kP_  * (angle_in_rad - yaw_)
        velocity_publisher_.publish(msg)
    
    
    msg.angular.z = 0.0

    velocity_publisher_.publish(msg)

    change_stage(4)
        

def go_to_door():

    global regions_, stage1_switch_

    if ( stage1_switch_ == True ):

            if regions_['one'] == 10:

                stage1_switch_ = False

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


def identify_door_number():

    global total_distance_, detected_door_number_, distance_to_door_from_start_

    print ("")

    if total_distance_ > 0 and total_distance_ < 2:

        detected_door_number_ = 1
        distance_to_door_from_start_ = 2
        print ("Door 1 Detected. Total distance travelled: {}".format(total_distance_))

    elif total_distance_ > 2 and total_distance_ < 4:

        detected_door_number_ = 2
        distance_to_door_from_start_ = 4
        print ("Door 2 Detected. Total distance travelled: {}".format(total_distance_))
    
    elif total_distance_ > 4 and total_distance_ < 6:

        detected_door_number_ = 3
        distance_to_door_from_start_ = 6
        print ("Door 3 Detected. Total distance travelled: {}".format(total_distance_))

    elif total_distance_ > 6 and total_distance_ < 8:

        detected_door_number_ = 4
        distance_to_door_from_start_ = 8
        print ("Door 4 Detected. Total distance travelled: {}".format(total_distance_))
    else:

        print ("Error")


    print ("")



def main():

    #Declare variables

    global velocity_publisher_, laser_subscriber_, odom_subscriber_, stage_, total_distance_, t0_, distance_to_door_from_start_


    #Initialize ros node

    rospy.init_node('ris', anonymous=True, disable_signals=True)

    velocity_publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    laser_subscriber_ = rospy.Subscriber('/scan', LaserScan, laserCallback)

    odom_subscriber_ = rospy.Subscriber('/odom', Odometry, odomCallback)

    rate = rospy.Rate(10)

    #Run the node until user exists

    while not rospy.is_shutdown():

        if ( stage_ == 1 ):

            go_to_door()

        elif ( stage_ == 2 ):

            identify_door_number()
            t0_ = rospy.Time.now().to_sec()
            move_forward_by_distance((distance_to_door_from_start_ - total_distance_), 0, 3)
                
        elif ( stage_ == 3):

            rotate(-90)

        elif (stage_ == 4):

            t0_ = rospy.Time.now().to_sec()
            move_forward_by_distance(2.0, -90, 5)

        elif (stage_ == 5):

            rospy.signal_shutdown("Done")


        rate.sleep()


if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
