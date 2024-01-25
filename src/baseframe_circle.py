#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import numpy as np


def move():
    rospy.init_node('velocity_publisher')
    
    pub_front_right_steering = rospy.Publisher('/w_bot/front_right_steering_controller/command', Float64, queue_size=2) 
    pub_rear_right_steering = rospy.Publisher('/w_bot/rear_right_steering_controller/command', Float64, queue_size=2) 
    pub_rear_left_steering = rospy.Publisher('/w_bot/rear_left_steering_controller/command', Float64, queue_size=2)
    pub_front_left_steering = rospy.Publisher('/w_bot/front_left_steering_controller/command', Float64, queue_size=2)
    
    pub_front_right_wheel = rospy.Publisher('/w_bot/front_right_wheel_controller/command', Float64, queue_size=2)
    pub_rear_right_wheel = rospy.Publisher('/w_bot/rear_right_wheel_controller/command', Float64, queue_size=2)
    pub_rear_left_wheel = rospy.Publisher('/w_bot/rear_left_wheel_controller/command', Float64, queue_size=2)
    pub_front_left_wheel = rospy.Publisher('/w_bot/front_left_wheel_controller/command', Float64, queue_size=2)
    
    rate=rospy.Rate(2)
    # Get user Inputs as Distance and time
    print("Baseframe moves in vx and vy and rotates about z")
    
    # Set current time and current distance
    
    current_distance = 0
    radius=8
    circle_w=0.4
    L=0.420
    W=0.260

    t0= rospy.Time.now().to_sec()

    #Now, 
    while not rospy.is_shutdown():
        t=rospy.Time.now().to_sec()
        print(t)
        vx=-circle_w*radius*np.sin(circle_w*(t-t0))
        vy=circle_w*radius*np.cos(circle_w*(t-t0))
        omega=0
        
        v1x=float(vx) + float(omega)*L/2
        v1y=float(vy) - float(omega)*W/2
        v2x=float(vx) + float(omega)*L/2
        v2y=float(vy) + float(omega)*W/2
        v3x=float(vx) - float(omega)*L/2
        v3y=float(vy) + float(omega)*W/2
        v4x=float(vx) - float(omega)*L/2
        v4y=float(vy) - float(omega)*W/2
        
        
        pub_front_right_steering.publish(math.atan2(v1x,v1y))
        pub_front_left_steering.publish(math.atan2(v2x,v2y))
        pub_rear_left_steering.publish(math.atan2(v3x,v3y))
        pub_rear_right_steering.publish(math.atan2(v4x,v4y))
        
        pub_front_right_wheel.publish(math.sqrt(v1x*v1x + v1y*v1y))
        pub_front_left_wheel.publish(math.sqrt(v2x*v2x + v2y*v2y))
        pub_rear_left_wheel.publish(math.sqrt(v3x*v3x + v3y*v3y))
        pub_rear_right_wheel.publish(math.sqrt(v4x*v4x + v4y*v4y))
        
        rate.sleep()
    
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
    
    
