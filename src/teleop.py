#!/usr/bin/env python
import sympy as sym
import numpy as num

import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import Float64




a, d, alpha, theta = sym.symbols('a, d, alpha, theta')
a0, d0, alpha0, theta0 = sym.symbols('a0, d0, alpha0, theta0')
a1, d1, alpha1, theta1 = sym.symbols('a1, d1, alpha1, theta1')
a2, d2, alpha2, theta2 = sym.symbols('a2, d2, alpha2, theta2')
a3, d3, alpha3, theta3 = sym.symbols('a3, d3, alpha3, theta3')
a4, d4, alpha4, theta4 = sym.symbols('a4, d4, alpha4, theta4')
a5, d5, alpha5, theta5 = sym.symbols('a5, d5, alpha5, theta5')
a6, d6, alpha6, theta6 = sym.symbols('a6, d6, alpha6, theta6')
a7, d7, alpha7, theta7 = sym.symbols('a7, d7, alpha7, theta7')

a1 = 0.033
a2 = 0.155
a3 = 0.135
a4 = 0
a5 = 0
# a6 = 0.088
# a7 = 0 

d1 = 0.147
d2 = 0
d3 = 0
d4 = 0
d5 = 0.217
# d6 = 0
# d7 = -0.207

alpha1 = -sym.pi/2.0
alpha2 = 0
alpha3 = 0
alpha4 = sym.pi/2.0
alpha5 = 0
# alpha6 = -sym.pi/2.0
# alpha7 = 0


# theta3 = 0 # As 3rd joint is locked, we give constant value for theta3

T = 4

x_final = 3
y_final = 3
z_final = 1
theta_final = sym.pi/2
phi_final =  0
psi_final =  0


#Transformation matrix for 1st joint
A1 = sym.Matrix([[sym.cos(theta1), -sym.sin(theta1)*sym.cos(alpha1), sym.sin(theta1)*sym.sin(alpha1), a1*sym.cos(theta1)],
   [sym.sin(theta1), sym.cos(theta1)*sym.cos(alpha1), -sym.cos(theta1)*sym.sin(alpha1), a1*sym.sin(theta1)],
   [0, sym.sin(alpha1), sym.cos(alpha1), d1],
   [0, 0, 0, 1]])

#Transformation matrix for 2nd joint
A2 = sym.Matrix([[sym.cos(theta2-sym.pi/2 ), -sym.sin(theta2-sym.pi/2 )*sym.cos(alpha2), sym.sin(theta2-sym.pi/2 )*sym.sin(alpha2), a2*sym.cos(theta2-sym.pi/2 )],
   [sym.sin(theta2-sym.pi/2 ), sym.cos(theta2-sym.pi/2 )*sym.cos(alpha2), -sym.cos(theta2-sym.pi/2 )*sym.sin(alpha2), a2*sym.sin(theta2-sym.pi/2 )],
   [0, sym.sin(alpha2), sym.cos(alpha2), d2],
   [0, 0, 0, 1]])

#Transformation matrix for 3rd joint
A3 = sym.Matrix([[sym.cos(theta3), -sym.sin(theta3)*sym.cos(alpha3), sym.sin(theta3)*sym.sin(alpha3), a3*sym.cos(theta3)],
   [sym.sin(theta3), sym.cos(theta3)*sym.cos(alpha3), -sym.cos(theta3)*sym.sin(alpha3), a3*sym.sin(theta3)],
   [0, sym.sin(alpha3), sym.cos(alpha3), d3],
   [0, 0, 0, 1]])

#Transformation matrix for 4th joint
A4 = sym.Matrix([[sym.cos(theta4+sym.pi/2), -sym.sin(theta4+sym.pi/2)*sym.cos(alpha4), sym.sin(theta4+sym.pi/2)*sym.sin(alpha4), a4*sym.cos(theta4+sym.pi/2)],
   [sym.sin(theta4+sym.pi/2), sym.cos(theta4+sym.pi/2)*sym.cos(alpha4), -sym.cos(theta4+sym.pi/2)*sym.sin(alpha4), a4*sym.sin(theta4+sym.pi/2)],
   [0, sym.sin(alpha4), sym.cos(alpha4), d4],
   [0, 0, 0, 1]])

#Transformation matrix for 5th joint
A5 = sym.Matrix([[sym.cos(theta5), -sym.sin(theta5)*sym.cos(alpha5), sym.sin(theta5)*sym.sin(alpha5), a5*sym.cos(theta5)],
   [sym.sin(theta5), sym.cos(theta5)*sym.cos(alpha5), -sym.cos(theta5)*sym.sin(alpha5), a5*sym.sin(theta5)],
   [0, sym.sin(alpha5), sym.cos(alpha5), d5],
   [0, 0, 0, 1]])

#Transformation matrix for 6th joint
A6 = sym.Matrix([[sym.cos(theta6), -sym.sin(theta6)*sym.cos(alpha6), sym.sin(theta6)*sym.sin(alpha6), a6*sym.cos(theta6)],
   [sym.sin(theta6), sym.cos(theta6)*sym.cos(alpha6), -sym.cos(theta6)*sym.sin(alpha6), a6*sym.sin(theta6)],
   [0, sym.sin(alpha6), sym.cos(alpha6), d6],
   [0, 0, 0, 1]]) 

#Transformation matrix for 7th joint
A7 = sym.Matrix([[sym.cos(theta7), -sym.sin(theta7)*sym.cos(alpha7), sym.sin(theta7)*sym.sin(alpha7), a7*sym.cos(theta7)],
   [sym.sin(theta7), sym.cos(theta7)*sym.cos(alpha7), -sym.cos(theta7)*sym.sin(alpha7), a7*sym.sin(theta7)],
   [0, sym.sin(alpha7), sym.cos(alpha7), d7],
   [0, 0, 0, 1]]) 
 
# A is the overall transformation joint 
A = A1*A2*A3*A4*A5
Origin = sym.Matrix([[0], [0], [0], [1]])
End_effector_vector = A*Origin

#Transformation matrices for each link
A0_1 = A1
A0_2 = A1*A2
A0_3 = A1*A2*A3
A0_4 = A1*A2*A3*A4
A0_5 = A1*A2*A3*A4*A5
A0_6 = A1*A2*A3*A4*A5*A6
A0_7 = A1*A2*A3*A4*A5*A6*A7

#Extracting Z vector component from each transformation matrix 
Z_vector = sym.Matrix([[0], [0], [1], [0]])

Z0_1 = A0_1*Z_vector
Z0_2 = A0_2*Z_vector
Z0_3 = A0_3*Z_vector
Z0_4 = A0_4*Z_vector
Z0_5 = A0_5*Z_vector
Z0_6 = A0_6*Z_vector
Z0_7 = A0_7*Z_vector

#Differentiating Z matricies w.r.t all joint angles
H0_1 = End_effector_vector.diff(theta1)

H0_2 = End_effector_vector.diff(theta2)

H0_3 = End_effector_vector.diff(theta3)

H0_4 = End_effector_vector.diff(theta4)

H0_5 = End_effector_vector.diff(theta5)

H0_6 = End_effector_vector.diff(theta6)

H0_7 = End_effector_vector.diff(theta7)

#Concatanting all H matricies to get upper half of Jacobian
J_upper = H0_1
J_upper = J_upper.row_join(H0_2)
J_upper = J_upper.row_join(H0_3)
J_upper = J_upper.row_join(H0_4)
J_upper = J_upper.row_join(H0_5)
# J_upper = J_upper.row_join(H0_6)
# J_upper = J_upper.row_join(H0_7)

J_upper.row_del(3)

#Concatanting all Z matricies to get lower half of Jacobian
J_lower = Z0_1
J_lower = J_lower.row_join(Z0_2)
J_lower = J_lower.row_join(Z0_3)
J_lower = J_lower.row_join(Z0_4)
J_lower = J_lower.row_join(Z0_5)
# J_lower = J_lower.row_join(Z0_6)
# J_lower = J_lower.row_join(Z0_7)

J_lower.row_del(3)

#Concatanting all upper and lower halves to get upper Jacobian
J = J_upper.col_join(J_lower)
# sym.pprint(J)

# Substituting theta values to get value of Jacobian
J_val = J.subs({theta1: 0, theta2: 0,theta3 : 0, theta4 : 0, theta5: 0})

#Taking inverse of Jacobian
J_val = num.array(J_val)
print("here")
print(J_val)
J_val = J_val.astype(num.float32)
J_inv = num.linalg.pinv(J_val)

# Finding position of pen by susbtituting values in A matrix 
Asub = A.subs({theta1: 0, theta2: 0,theta3 : 0, theta4 : sym.pi/2, theta5: 0})
End_effector_vector = Asub*Origin

x_plot = [End_effector_vector[0]]
y_plot = [End_effector_vector[1]]
z_plot = [End_effector_vector[2]]


x_initial = End_effector_vector[0]
y_initial = End_effector_vector[1]
z_initial = End_effector_vector[2]
theta_initial =  0
phi_initial =  0
psi_initial =  0

#Defining initial values
r = 0.1
omega = 2*num.pi/50
v_x_dot = 0.0
y_dot = 0.0
z_dot = 0.0
theta_dot = 0.0
phi_dot = 0.0
psi_dot = 0.0

theta6_val = sym.pi
theta7_val = 0



def go_to_point(x_final,y_final,z_final):

    Asub = A.subs({theta1: 0, theta2: 0,theta3 : 0, theta4 : 0, theta5: 0})
    End_effector_vector = Asub*Origin

    x_initial = End_effector_vector[0]
    y_initial = End_effector_vector[1]
    z_initial = End_effector_vector[2]

    tol = 0.005
    theta1_val = 0
    theta2_val = 0
    theta3_val = 0
    theta4_val = 0
    theta5_val = 0
    

    global J_inv
    x_now = x_initial
    y_now = y_initial
    z_now = z_initial
    theta_now = theta_initial
    phi_now = phi_initial
    psi_now = psi_initial
    print("z_now",z_now)
    print("y_now",y_now)
    print("x_now",x_now)
    t = 0
    # for t in range(0, 500):
    while 1:
        #Computing inverse kinematics to get change in joint angles
        # v_x_dot = (x_final - x_initial)/T
        # y_dot = (y_final - y_initial)/T
        # z_dot = (z_final - z_initial)/T
        if x_final > x_now:
            v_x_dot = 0.002
        elif x_final < x_now:
            v_x_dot = -0.002
        else:
            v_x_dot = 0.00 

        if y_final > y_now:
            y_dot = 0.0
        elif y_final < y_now:
            y_dot = -0.002
        else:
            y_dot = 0.00 
        if z_final > z_now:
            z_dot = 0.001
        elif z_final < z_now:
            z_dot = -0.001
        else:
            z_dot = 0.00     
        theta_dot = 0
        phi_dot = 0
        psi_dot = 0

        key = getKey() 
        if key == 'k' :
            break
        # theta_dot = (theta_final - theta_now)/(T-t/100)   
        # phi_dot = (phi_final - phi_now)/(T-t/100)
        # psi_dot = (psi_final - psi_now)/(T-t/100)

        J_val = J.subs({theta1: theta1_val, theta2: theta2_val,theta3 : theta3_val, theta4 : theta4_val, theta5: theta5_val})
        J_val = num.array(J_val)
        
        J_val = J_val.astype(num.float32)
        J_inv = num.linalg.pinv(J_val)        
        
        X_dot = sym.Matrix([[v_x_dot],[y_dot],[z_dot],[theta_dot],[phi_dot],[psi_dot]])
        q_dot =num.dot(J_inv, X_dot)

        if(q_dot[0][0]>(2.84)):
            q_dot[0][0] = 0.01
        if(q_dot[1][0]>(1.57)):
            q_dot[1][0] = 0.01
        if(q_dot[2][0]>(2.54)):
            q_dot[2][0] = 0.01
        if(q_dot[3][0]>(1.78)):
            q_dot[3][0] = 0.01
        if(q_dot[4][0]>(2.92)):
            q_dot[4][0] = 0.01
            
        if(q_dot[0][0]<(-2.84)):
            q_dot[0][0] = -0.01
        if(q_dot[1][0]<(-1.13)):
            q_dot[1][0] = -0.01
        if(q_dot[2][0]<(-2.61)):
            q_dot[2][0] = -0.01
        if(q_dot[3][0]<(-1.78)):
            q_dot[3][0] = -0.01
        if(q_dot[4][0]<(-2.92)):
            q_dot[4][0] = -0.01

        #Calcualting joint angles based on the change in joint angles and their previous values
        theta1_val = theta1_val + q_dot[0][0]
        theta2_val = theta2_val + q_dot[1][0]
        theta3_val = theta3_val + q_dot[2][0]
        theta4_val = theta4_val + q_dot[3][0]
        theta5_val = theta5_val + q_dot[4][0]

        
        # Then Computing forward kinematics to get position of pen as per calucalted joint angles
        Asub = A.subs({theta1: theta1_val, theta2: theta2_val + sym.pi/2, theta3: theta3_val, theta4 : theta4_val + sym.pi/2, theta5: theta5_val})
        End_effector_vector = Asub*Origin

        x_now = End_effector_vector[0]
        y_now = End_effector_vector[1]
        z_now = End_effector_vector[2]
        
        if (x_final - tol < x_now < x_final + tol) and (y_final - tol < y_now < y_final + tol ) and (z_final - tol < z_now < z_final + tol ) : 
            break

        pub_joint1.publish(float(theta1_val))
        pub_joint2.publish(float(theta2_val))
        pub_joint3.publish(float(theta3_val))
        pub_joint4.publish(float(theta4_val))
        pub_joint5.publish(float(theta5_val))
        
        t += 1

        if t>300:
            break
        
        #Again computing Jacobian and its inverse for the new joint angles

        

    #Converting stored points' list into float arrays for ploting purpose
    xdata = num.array(x_plot, dtype=num.float32)
    ydata = num.array(y_plot, dtype=num.float32)
    zdata = num.array(z_plot, dtype=num.float32)


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
#                 go_to_point(0.3,0.2,0.12)
                pub_joint1.publish(1.57)
                pub_joint2.publish(0.7)
                pub_joint3.publish(1.15)
                pub_joint4.publish(0)
                pub_joint5.publish(0.5)
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
