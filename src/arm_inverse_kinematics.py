#!/usr/bin/env python

from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import Float64
# declare symbols
a,d, alpha,theta, t1, t2, t3, t4, t5, t6, t7 = sym.symbols('a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6 theta7')

# Find transformation matrix from given DH parameters
def FindTransformationMatrix(a,d,alpha,theta):
    transformation_matrix = sym.Matrix(
        [[sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), a*sym.cos(theta)], 
         [sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
         [0, sym.sin(alpha), sym.cos(alpha), d],
         [0, 0, 0, 1]])

    return transformation_matrix


# Declaring link lengths as symbols
d1,d3,d5,d7,a3 = sym.symbols('d1, d3, d5, d7, a3')

def go_to_point():

    # Transformation matrices of each frame with respect to previous frames(A1 to A6) and Dummy frame
    A1=FindTransformationMatrix(0.033,0.147,-sym.pi/2,t1)
    A2=FindTransformationMatrix(0.155,0,0,t2-sym.pi/2)
    A3=FindTransformationMatrix(0.135,0,0,t3)
    A4=FindTransformationMatrix(0,0,sym.pi/2,t4+sym.pi/2)
    A5=FindTransformationMatrix(0,0.171,0,t5)


    # Transformation matrix of each frame with respect to groud frame(T0_i)
    T0_1=A1
    T0_2=A1*A2
    T0_3=A1*A2*A3
    T0_4=A1*A2*A3*A4
    T0_5=A1*A2*A3*A4*A5

    sym.simplify(T0_5)
    T0_5_curr=T0_5.subs([(t1, 0), (t2, 0), (t3, 0), (t4, 0), (t5, 0)])
    # sym.pprint(T0_5_curr.col(3))

    # Find Z0_i for each matrix computed above
    Z0_1=T0_1.col(2)
    Z0_1.row_del(3)

    Z0_2=T0_2.col(2)
    Z0_2.row_del(3)

    Z0_3=T0_3.col(2)
    Z0_3.row_del(3)

    Z0_4=T0_4.col(2)
    Z0_4.row_del(3)

    Z0_5=T0_5.col(2)
    Z0_5.row_del(3)

    X0_5=T0_5.col(3)
    X0_5.row_del(3)

    # Find partial derivate of X0_6 w.r.t each joint angle
    M1=X0_5.diff(t1)
    M2=X0_5.diff(t2)
    M3=X0_5.diff(t3)
    M4=X0_5.diff(t4)
    M5=X0_5.diff(t5)

    # Find jacobian matrix using second method
    Jacobian=sym.Matrix([[M1,M2,M3,M4,M5],
                         [Z0_1,Z0_2,Z0_3,Z0_4,Z0_5]])

    print("Jacobian Matrix =")
#     sym.pprint(Jacobian)

    # Let's plot the trajectory now
    num_data_points=48
    # Trajectrory need to be plotted within 5 sec
    Time=5
    delta_t=Time/num_data_points

    # r*w = perimeter/Total Time =2*pi*r/Time
    t=sym.symbols('t')
    w=2*sym.pi/Time
    r=0.1
    k=0.337

    # Parametric equation of a circle
    x=0.204
    y=r*sym.sin(w*t)
    z=k+r*sym.cos(w*t)

    # Linear velocities w.r.t time. Angular velocities are zero
    time1,time2=sym.symbols('time1 time2')
    x_dot=0
    y_dot=r*(sym.sin(w*time2)-sym.sin(w*time1))/delta_t
    z_dot=r*(sym.cos(w*time2)-sym.cos(w*time1))/delta_t


    # Set time to 0. Populate initial position vector and joint angle vector
    time=0

    q=sym.Matrix([[0.0000, 0.0000, 0.000, sym.pi/2, 0.0000]])
    q=q.transpose()

    X=sym.Matrix([[0.204, 0.0, 0.437, 0.0, 0, 0]])
    X=X.transpose()

    X_DOT=sym.Matrix([[0.0, y_dot, z_dot, 0.0, 0.0, 0.0]])
    X_DOT=X_DOT.transpose()

    # Populate coordinate vector for pen tip w.r.t. end effector frame
    pen_pos_wrt_end_effector=sym.Matrix([[0, 0, 0, 1]])
    pen_pos_wrt_end_effector=pen_pos_wrt_end_effector.transpose()

    ax = plt.axes(projection ="3d")

    # For loop used to plot data points one by one
    for i in range(0,num_data_points):
        # First plot the point using forward kinematics with known joint angles
        T0_5_curr=T0_5.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4))])
        pen_curr=T0_5_curr*pen_pos_wrt_end_effector
        
        print(pen_curr[0].evalf(4),',',pen_curr[1].evalf(4),',',pen_curr[2].evalf(4))
    #     plt.pause(1)
        
        # Compute joint angles for next point to be plotted
        Jacobian_curr=Jacobian.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4))])
        X_DOT_curr=X_DOT.subs([(time1,time),(time2,time+delta_t)])

        # Find Inverse of Jacobian
        Jacobian_curr=np.array(Jacobian_curr)
        Jacobian_curr=Jacobian_curr.astype(np.float32)
        Jacobian_inv=np.linalg.pinv(Jacobian_curr)
        
        pub_joint1.publish((q[0].evalf(4)))
        pub_joint2.publish((q[1].evalf(4)))
        pub_joint3.publish((q[2].evalf(4)))
        pub_joint4.publish((q[3].evalf(4)))
        pub_joint5.publish((q[4].evalf(4)))

        # Compute q_dot
        q_dot=Jacobian_inv*X_DOT_curr

        # Next q by numerical integration
        q=q+q_dot*delta_t
        
        # Increment time 
        time=time+delta_t

# Show plot
# plt.axis('equal')
# plt.show()



msg = """
Control Your Toy!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 40
turn = 3.14


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')

    pub_front_right_steering = rospy.Publisher('/w_bot/front_right_steering_controller/command', Float64, queue_size=2) 
    pub_rear_right_steering = rospy.Publisher('/w_bot/rear_right_steering_controller/command', Float64, queue_size=2) 
    pub_rear_left_steering = rospy.Publisher('/w_bot/rear_left_steering_controller/command', Float64, queue_size=2) 
    pub_front_left_steering = rospy.Publisher('/w_bot/front_left_steering_controller/command', Float64, queue_size=2) 
    
    pub_front_right_wheel = rospy.Publisher('/w_bot/front_right_wheel_controller/command', Float64, queue_size=2) 
    pub_rear_right_wheel = rospy.Publisher('/w_bot/rear_right_wheel_controller/command', Float64, queue_size=2) 
    pub_rear_left_wheel = rospy.Publisher('/w_bot/rear_left_wheel_controller/command', Float64, queue_size=2) 
    pub_front_left_wheel = rospy.Publisher('/w_bot/front_left_wheel_controller/command', Float64, queue_size=2) 
    
    pub_joint1 = rospy.Publisher('/w_bot/joint1_controller/command', Float64, queue_size=2)
    pub_joint2 = rospy.Publisher('/w_bot/joint2_controller/command', Float64, queue_size=2) 
    pub_joint3 = rospy.Publisher('/w_bot/joint3_controller/command', Float64, queue_size=2) 
    pub_joint4 = rospy.Publisher('/w_bot/joint4_controller/command', Float64, queue_size=2) 
    pub_joint5 = rospy.Publisher('/w_bot/joint5_controller/command', Float64, queue_size=2) 

    pub_left_finger = rospy.Publisher('/w_bot/finger_left_controller/command', Float64, queue_size=2)
    pub_right_finger = rospy.Publisher('/w_bot/finger_right_controller/command', Float64, queue_size=2) 
    
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print (msg)
        print (vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print (vels(speed,turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                pub_joint1.publish(0)
                pub_joint2.publish(0)
                pub_joint3.publish(0)
                pub_joint4.publish(0)
                pub_joint5.publish(0)
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.2)
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.2)
            else:
                control_speed = target_speed

            # if target_turn > control_turn:
            #     control_turn = min( target_turn, control_turn + 0.1)
            # elif target_turn < control_turn:
            #     control_turn = max( target_turn, control_turn - 0.1)
            # else:
            #     control_turn = target_turn

            if target_turn < control_turn:
                control_turn = target_turn
            else:
                control_turn=control_turn+0.1*1.57*th


            pub_front_right_steering.publish(control_turn)
            pub_rear_right_steering.publish(control_turn)
            pub_rear_left_steering.publish(control_turn)
            pub_front_left_steering.publish(control_turn)
    
            pub_front_right_wheel.publish(control_speed)
            pub_rear_right_wheel.publish(control_speed)
            pub_rear_left_wheel.publish(control_speed)
            pub_front_left_wheel.publish(control_speed)
            
            if key == 'b':
                go_to_point()
                # pub_joint1.publish(1.57)
                # pub_joint2.publish(0.7)
                # pub_joint3.publish(1.15)
                # pub_joint4.publish(0)
                # pub_joint5.publish(0.5)
            if key == 'g':
                pub_left_finger.publish(1)
                pub_right_finger.publish(1)
            if key == 'r':
                pub_left_finger.publish(-1)
                pub_right_finger.publish(0)
            
            print(control_speed,control_turn)

    except:
        print (e)

    finally:
        pub_front_right_steering.publish(control_turn)
        pub_rear_right_steering.publish(control_turn)
        pub_rear_left_steering.publish(control_turn)
        pub_front_left_steering.publish(control_turn)
    
        pub_front_right_wheel.publish(control_speed)
        pub_rear_right_wheel.publish(control_speed)
        pub_rear_left_wheel.publish(control_speed)
        pub_front_left_wheel.publish(control_speed)
        
        pub_joint1.publish(0)
        pub_joint2.publish(0)
        pub_joint3.publish(0)
        pub_joint4.publish(0)
        pub_joint5.publish(0)

        pub_left_finger.publish(1)
        pub_right_finger.publish(1)
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


