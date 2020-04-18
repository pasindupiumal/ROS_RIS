#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}


def callback(msg):

    global regions_

    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    print 'Distance from right wall -  %f' % regions_['right']



def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

    laser_sub = rospy.Subscriber('/scan', LaserScan, callback )
	
    vel_msg = Twist()
    
    
    speed = 0.10
    distance = 9.0
    isForward = True

    #Checking if the movement is forward or backwards
    if(isForward):
       vel_msg.linear.x = abs(speed)
    else:
       vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

       #Setting the current time for distance calculus
       t0 = rospy.Time.now().to_sec()
       current_distance = 0

       #Loop to move the turtle in an specified distance
       while(current_distance < distance):
           #Publish the velocity
           velocity_publisher.publish(vel_msg)
           #Takes actual time to velocity calculus
           t1=rospy.Time.now().to_sec()
           #Calculates distancePoseStamped
           current_distance= speed*(t1-t0)
       #After the loop, stops the robot
       vel_msg.linear.x = 0
       #Force the robot to stop
       velocity_publisher.publish(vel_msg)


def main():

    move()



if __name__ == '__main__':
    try:
       #Testing our function
       main()
    except rospy.ROSInterruptException: pass
