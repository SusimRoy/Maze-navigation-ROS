#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    regions = {
        'left':  min(min(msg.ranges[55:90]), 10),
        'fleft': min(min(msg.ranges[19:54]), 10),
        'front':  min(min(msg.ranges[-18:0]+msg.ranges[0:18]), 10),
        'fright':  min(min(msg.ranges[-54:-19]), 10),
        'right':   min(min(msg.ranges[-90:-55]), 10),
    }
    
    take_action(regions)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    d=0.69
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        linear_x = 0
        angular_z = -0.5
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = -0.5
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
    if regions['fright']<0.3:
        linear_x=0
        angular_z=0.5
    else:
        linear_x = 0.5
        angular_z = 0.5
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.5
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.5
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.5
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.5
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.5
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

    
rospy.init_node('laser_move')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
sub=rospy.Subscriber('/scan',LaserScan,callback)

rospy.spin()
