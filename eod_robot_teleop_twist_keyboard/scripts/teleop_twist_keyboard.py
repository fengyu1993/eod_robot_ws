#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

import math 

from eod_robot_msgs.msg import Eod_robot_twist

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
    Vehicle:
        q   w   e
        a   s   d
        z   x   c

    Left arm: 
        r   t   y   u   i   o
        f   g   h   j   k   l  

    Right arm: 
        R   T   Y   U   I   O
        F   G   H   J   K   L    
        V   B   N   M 

    Gripper:
        .  , 

Regulation:
    Vehicle:  
        7/1 : increase/decrease max speeds by 10%
        8/2 : increase/decrease only linear speed_vecihle by 10%
        9/3 : increase/decrease only angular speed_vecihle by 10%

    Left arm: 
        *// : increase/decrease speed_vecihle by 10%
          p : joint/task
    v/b/n/m : work/front/behind/stand pose

    Right arm: 
        +/- : increase/decrease speed_vecihle by 10%
          P : joint/task
    V/B/N/M : work/front/behind/stand pose

    Gripper:  
        >/< : increase/decrease only linear speed_vecihle by 10%

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'e':(1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'a':(0,1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'd':(0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'q':(1,1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'x':(-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'c':(-1,1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'z':(-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0),
        'r':(0,0,1,0,0,0,0,0,0,0,0,0,0,0,0),
        't':(0,0,0,1,0,0,0,0,0,0,0,0,0,0,0),
        'y':(0,0,0,0,1,0,0,0,0,0,0,0,0,0,0),
        'u':(0,0,0,0,0,1,0,0,0,0,0,0,0,0,0),
        'i':(0,0,0,0,0,0,1,0,0,0,0,0,0,0,0),
        'o':(0,0,0,0,0,0,0,1,0,0,0,0,0,0,0),
        'f':(0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0),
        'g':(0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0),
        'h':(0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0),
        'j':(0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0),
        'k':(0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0),
        'l':(0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0),    
        'R':(0,0,0,0,0,0,0,0,1,0,0,0,0,0,0),
        'T':(0,0,0,0,0,0,0,0,0,1,0,0,0,0,0),
        'Y':(0,0,0,0,0,0,0,0,0,0,1,0,0,0,0),
        'U':(0,0,0,0,0,0,0,0,0,0,0,1,0,0,0),
        'I':(0,0,0,0,0,0,0,0,0,0,0,0,1,0,0),
        'O':(0,0,0,0,0,0,0,0,0,0,0,0,0,1,0),
        'F':(0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0),
        'G':(0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0),
        'H':(0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0),
        'J':(0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0),
        'K':(0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0),
        'L':(0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0),   
        '.':(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1),
        ',':(0,0,0,0,0,0,0,0,0,0,0,0,0,0,1), 
    }

speedBindings = {
        '7':(1.1,1.1,1,1,1),
        '1':(0.9,0.9,1,1,1),
        '8':(1.1,1,1,1,1),
        '2':(0.9,1,1,1,1),
        '9':(1,1.1,1,1,1),
        '3':(1,0.9,1,1,1),
        '*':(1,1,1.1,1,1),
        '/':(1,1,0.9,1,1),
        '+':(1,1,1,1.1,1),
        '-':(1,1,1,0.9,1),
        '>':(1,1,1,1,1.1),
        '<':(1,1,1,1,0.9),
    }

arm_left_pose = {
        'v':(1,),
        'b':(2,),
        'n':(3,),
        'm':(4,),
    }

arm_right_pose = {
        'V':(1,),
        'B':(2,),
        'N':(3,),
        'M':(4,),
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

speed_vecihle = rospy.get_param('speed_vecihle')
turn_vecihle = rospy.get_param('turn_vecihle')
speed_arm_left = rospy.get_param('speed_arm_left')
speed_arm_right = rospy.get_param('speed_arm_right')
speed_gripper = rospy.get_param('speed_gripper')
arm_left_joint_or_task = rospy.get_param('arm_left_joint_or_task')
arm_right_joint_or_task = rospy.get_param('arm_right_joint_or_task')

def vels_vecihle(speed_vecihle, turn_vecihle):
    return "currently vecihle:\tspeed %s\tturn %s " % (speed_vecihle,turn_vecihle)

def arm_left_vels(speed_arm_left):
    return "currently left arm:\tspeed %s" % (speed_arm_left)

def arm_right_vels(speed_arm_right):
    return "currently right arm:\tspeed %s" % (speed_arm_right)

def gripper_vels(speed_gripper):
    return "currently gripper:\tspeed %s" % (speed_gripper)

def element_add(twist):
    return twist.linear.x + twist.linear.y + twist.linear.z + twist.angular.x + twist.angular.y + twist.angular.z

def arm_right_joint_task(flag):
    if flag == False:
        return "arm right control mode: task space"
    else:
        return "arm right control mode: joint space"

def arm_left_joint_task(flag):
    if flag == False:
        return "arm left control mode: task space"
    else:
        return "arm left control mode: joint space"

def arm_left_pose_print(num):
    if num == 1:
        return "arm left pose: work pose"
    elif num == 2:
        return "arm left pose: front stand pose"
    elif num == 3:
        return "arm left pose: behind front stand pose"
    elif num == 4:
        return "arm left pose: stand pose"
    
def arm_right_pose_print(num):
    if num == 1:
        return "arm right pose: work pose"
    elif num == 2:
        return "arm right pose: front stand pose"
    elif num == 3:
        return "arm right pose: behind front stand pose"
    elif num == 4:
        return "arm right pose: stand pose"



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('cmd_eod_robot_vel', Eod_robot_twist, queue_size = 5)

    x_vecihle = 0
    th_vecihle = 0
    target_speed_vecihle = 0
    target_turn_vecihle = 0
    control_speed_vecihle = 0
    control_turn_vecihle = 0

    arm_left_S = Twist(); target_arm_left_twist = Twist(); control_arm_left_twist = Twist()
    arm_left_S.linear.x = 0; arm_left_S.linear.y = 0; arm_left_S.linear.z = 0
    arm_left_S.angular.x = 0; arm_left_S.angular.y = 0; arm_left_S.angular.z = 0
    target_arm_left_twist.linear.x = 0; target_arm_left_twist.linear.y = 0; target_arm_left_twist.linear.z = 0
    target_arm_left_twist.angular.x = 0; target_arm_left_twist.angular.y = 0; target_arm_left_twist.angular.z = 0  
    control_arm_left_twist.linear.x = 0; control_arm_left_twist.linear.y = 0; control_arm_left_twist.linear.z = 0
    control_arm_left_twist.angular.x = 0; control_arm_left_twist.angular.y = 0; control_arm_left_twist.angular.z = 0 

    arm_right_S = Twist(); target_arm_right_twist = Twist(); control_arm_right_twist = Twist()
    arm_right_S.linear.x = 0; arm_right_S.linear.y = 0; arm_right_S.linear.z = 0
    arm_right_S.angular.x = 0; arm_right_S.angular.y = 0; arm_right_S.angular.z = 0
    target_arm_right_twist.linear.x = 0; target_arm_right_twist.linear.y = 0; target_arm_right_twist.linear.z = 0
    target_arm_right_twist.angular.x = 0; target_arm_right_twist.angular.y = 0; target_arm_right_twist.angular.z = 0  
    control_arm_right_twist.linear.x = 0; control_arm_right_twist.linear.y = 0; control_arm_right_twist.linear.z = 0
    control_arm_right_twist.angular.x = 0; control_arm_right_twist.angular.y = 0; control_arm_right_twist.angular.z = 0  

    x_gripper = 0
    target_speed_gripper = 0
    control_speed_gripper = 0

    status = 0
    count = 0

    try:
        print(msg)
        print(vels_vecihle(speed_vecihle,turn_vecihle))
        print(arm_left_vels(speed_arm_left))
        print(arm_right_vels(speed_arm_right))
        print(gripper_vels(speed_gripper))
        print(arm_left_joint_task(arm_left_joint_or_task))
        print(arm_right_joint_task(arm_right_joint_or_task))

        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x_vecihle = moveBindings[key][0]
                th_vecihle = moveBindings[key][1]

                arm_left_S.linear.x = moveBindings[key][2]
                arm_left_S.linear.y = moveBindings[key][3]
                arm_left_S.linear.z = moveBindings[key][4]
                arm_left_S.angular.x = moveBindings[key][5]
                arm_left_S.angular.y = moveBindings[key][6]
                arm_left_S.angular.z = moveBindings[key][7]

                arm_right_S.linear.x = moveBindings[key][8]
                arm_right_S.linear.y = moveBindings[key][9]
                arm_right_S.linear.z = moveBindings[key][10]
                arm_right_S.angular.x = moveBindings[key][11]
                arm_right_S.angular.y = moveBindings[key][12]
                arm_right_S.angular.z = moveBindings[key][13]

                x_gripper = moveBindings[key][14]

                # print("x_vecihle = %s, th_vecihle = %s" % (x_vecihle, th_vecihle))
                # print("arm_left_S = [%s, %s, %s, %s, %s, %s]" % \
                # (arm_left_S.linear.x, arm_left_S.linear.y, arm_left_S.linear.z, \
                # arm_left_S.angular.x, arm_left_S.angular.y, arm_left_S.angular.z))
                # print("arm_right_S = [%s, %s, %s, %s, %s, %s]" % \
                # (arm_right_S.linear.x, arm_right_S.linear.y, arm_right_S.linear.z, \
                # arm_right_S.angular.x, arm_right_S.angular.y, arm_right_S.angular.z))
                # print("x_gripper = %s" % (x_gripper))
            elif key in speedBindings.keys():
                speed_vecihle = speed_vecihle * speedBindings[key][0]
                turn_vecihle = turn_vecihle * speedBindings[key][1]
                speed_arm_left = speed_arm_left * speedBindings[key][2]
                speed_arm_right = speed_arm_right * speedBindings[key][3]
                speed_gripper = speed_gripper * speedBindings[key][4]

                count = 0

                print(vels_vecihle(speed_vecihle,turn_vecihle))
                print(arm_left_vels(speed_arm_left))
                print(arm_right_vels(speed_arm_right))
                print(gripper_vels(speed_gripper))
                if (status == 14):
                    print(msg)
                    print(vels_vecihle(speed_vecihle,turn_vecihle))
                    print(arm_left_vels(speed_arm_left))
                    print(arm_right_vels(speed_arm_right))
                    print(gripper_vels(speed_gripper))
                    print(arm_left_joint_task(arm_left_joint_or_task))
                    print(arm_right_joint_task(arm_right_joint_or_task))
                status = (status + 1) % 15
            elif key in arm_left_pose.keys():
                num = arm_left_pose[key][0]

                rospy.set_param('arm_left_pose_num', num)

                print(arm_left_pose_print(num))
            elif key in arm_right_pose.keys():
                num = arm_right_pose[key][0]

                rospy.set_param('arm_right_pose_num', num)

                print(arm_right_pose_print(num))
            elif key == '':
                x_vecihle = 0
                th_vecihle = 0
                control_speed_vecihle = 0

                arm_left_S.linear.x = 0; arm_left_S.linear.y = 0; arm_left_S.linear.z = 0
                arm_left_S.angular.x = 0; arm_left_S.angular.y = 0; arm_left_S.angular.z = 0 
                control_arm_left_twist.linear.x = 0; control_arm_left_twist.linear.y = 0; control_arm_left_twist.linear.z = 0
                control_arm_left_twist.angular.x = 0; control_arm_left_twist.angular.y = 0; control_arm_left_twist.angular.z = 0
                
                arm_right_S.linear.x = 0; arm_right_S.linear.y = 0; arm_right_S.linear.z = 0
                arm_right_S.angular.x = 0; arm_right_S.angular.y = 0; arm_right_S.angular.z = 0 
                control_arm_right_twist.linear.x = 0; control_arm_right_twist.linear.y = 0; control_arm_right_twist.linear.z = 0
                control_arm_right_twist.angular.x = 0; control_arm_right_twist.angular.y = 0; control_arm_right_twist.angular.z = 0 

                x_gripper = 0
                control_speed_gripper = 0
            elif (key == 'p'):
                arm_left_joint_or_task = not arm_left_joint_or_task

                print(arm_left_joint_task(arm_left_joint_or_task))
            elif (key == 'P'):
                arm_right_joint_or_task = not arm_right_joint_or_task

                print(arm_right_joint_task(arm_right_joint_or_task))
            elif (key == '\x03'):
                    break
        
            target_speed_vecihle = speed_vecihle * x_vecihle
            target_turn_vecihle = turn_vecihle * th_vecihle

            target_arm_left_twist.linear.x = speed_arm_left * arm_left_S.linear.x;   target_arm_left_twist.linear.y = speed_arm_left * arm_left_S.linear.y; target_arm_left_twist.linear.z = speed_arm_left * arm_left_S.linear.z
            target_arm_left_twist.angular.x = speed_arm_left * arm_left_S.angular.x;   target_arm_left_twist.angular.y = speed_arm_left * arm_left_S.angular.y; target_arm_left_twist.angular.z = speed_arm_left * arm_left_S.angular.z 

            target_arm_right_twist.linear.x = speed_arm_right * arm_right_S.linear.x;   target_arm_right_twist.linear.y = speed_arm_right * arm_right_S.linear.y; target_arm_right_twist.linear.z = speed_arm_right * arm_right_S.linear.z
            target_arm_right_twist.angular.x = speed_arm_right * arm_right_S.angular.x;   target_arm_right_twist.angular.y = speed_arm_right * arm_right_S.angular.y; target_arm_right_twist.angular.z = speed_arm_right * arm_right_S.angular.z 

            target_speed_gripper  = speed_gripper  * x_gripper 

            if target_speed_vecihle > control_speed_vecihle:
                control_speed_vecihle = min( target_speed_vecihle, control_speed_vecihle + 0.02 )
            elif target_speed_vecihle < control_speed_vecihle:
                control_speed_vecihle = max( target_speed_vecihle, control_speed_vecihle - 0.02 )
            else:
                control_speed_vecihle = target_speed_vecihle

            if target_turn_vecihle > control_turn_vecihle:
                control_turn_vecihle = min( target_turn_vecihle, control_turn_vecihle + 0.1 )
            elif target_turn_vecihle < control_turn_vecihle:
                control_turn_vecihle = max( target_turn_vecihle, control_turn_vecihle - 0.1 )
            else:
                control_turn_vecihle = target_turn_vecihle 

            if element_add(target_arm_left_twist) > element_add(control_arm_left_twist):
                if element_add(target_arm_left_twist) > element_add(control_arm_left_twist) + 0.02:
                    if(arm_left_S.linear.x != 0):
                        control_arm_left_twist.linear.x = control_arm_left_twist.linear.x + 0.02
                    if(arm_left_S.linear.y != 0):
                        control_arm_left_twist.linear.y = control_arm_left_twist.linear.y + 0.02
                    if(arm_left_S.linear.z != 0):
                        control_arm_left_twist.linear.z = control_arm_left_twist.linear.z + 0.02    
                    if(arm_left_S.angular.x != 0):
                        control_arm_left_twist.angular.x = control_arm_left_twist.angular.x + 0.02
                    if(arm_left_S.angular.y != 0):
                        control_arm_left_twist.angular.y = control_arm_left_twist.angular.y + 0.02
                    if(arm_left_S.angular.z != 0):
                        control_arm_left_twist.angular.z = control_arm_left_twist.angular.z + 0.02   
                else:
                    control_arm_left_twist.linear.x = target_arm_left_twist.linear.x
                    control_arm_left_twist.linear.y = target_arm_left_twist.linear.y
                    control_arm_left_twist.linear.z = target_arm_left_twist.linear.z
                    control_arm_left_twist.angular.x = target_arm_left_twist.angular.x
                    control_arm_left_twist.angular.y = target_arm_left_twist.angular.y
                    control_arm_left_twist.angular.z = target_arm_left_twist.angular.z
            elif element_add(target_arm_left_twist) < element_add(control_arm_left_twist):
                if element_add(target_arm_left_twist) > element_add(control_arm_left_twist) - 0.02:
                    control_arm_left_twist.linear.x = target_arm_left_twist.linear.x
                    control_arm_left_twist.linear.y = target_arm_left_twist.linear.y
                    control_arm_left_twist.linear.z = target_arm_left_twist.linear.z
                    control_arm_left_twist.angular.x = target_arm_left_twist.angular.x
                    control_arm_left_twist.angular.y = target_arm_left_twist.angular.y
                    control_arm_left_twist.angular.z = target_arm_left_twist.angular.z
                else:
                    if(arm_left_S.linear.x != 0):
                        control_arm_left_twist.linear.x = control_arm_left_twist.linear.x - 0.02
                    if(arm_left_S.linear.y != 0):
                        control_arm_left_twist.linear.y = control_arm_left_twist.linear.y - 0.02
                    if(arm_left_S.linear.z != 0):
                        control_arm_left_twist.linear.z = control_arm_left_twist.linear.z - 0.02    
                    if(arm_left_S.angular.x != 0):
                        control_arm_left_twist.angular.x = control_arm_left_twist.angular.x - 0.02
                    if(arm_left_S.angular.y != 0):
                        control_arm_left_twist.angular.y = control_arm_left_twist.angular.y - 0.02
                    if(arm_left_S.angular.z != 0):
                        control_arm_left_twist.angular.z = control_arm_left_twist.angular.z - 0.02 
            else:
                    control_arm_left_twist.linear.x = target_arm_left_twist.linear.x
                    control_arm_left_twist.linear.y = target_arm_left_twist.linear.y
                    control_arm_left_twist.linear.z = target_arm_left_twist.linear.z
                    control_arm_left_twist.angular.x = target_arm_left_twist.angular.x
                    control_arm_left_twist.angular.y = target_arm_left_twist.angular.y
                    control_arm_left_twist.angular.z = target_arm_left_twist.angular.z
            
            if element_add(target_arm_right_twist) > element_add(control_arm_right_twist):
                if element_add(target_arm_right_twist) > element_add(control_arm_right_twist) + 0.02:
                    if(arm_right_S.linear.x != 0):
                        control_arm_right_twist.linear.x = control_arm_right_twist.linear.x + 0.02
                    if(arm_right_S.linear.y != 0):
                        control_arm_right_twist.linear.y = control_arm_right_twist.linear.y + 0.02
                    if(arm_right_S.linear.z != 0):
                        control_arm_right_twist.linear.z = control_arm_right_twist.linear.z + 0.02    
                    if(arm_right_S.angular.x != 0):
                        control_arm_right_twist.angular.x = control_arm_right_twist.angular.x + 0.02
                    if(arm_right_S.angular.y != 0):
                        control_arm_right_twist.angular.y = control_arm_right_twist.angular.y + 0.02
                    if(arm_right_S.angular.z != 0):
                        control_arm_right_twist.angular.z = control_arm_right_twist.angular.z + 0.02   
                else:
                    control_arm_right_twist.linear.x = target_arm_right_twist.linear.x
                    control_arm_right_twist.linear.y = target_arm_right_twist.linear.y
                    control_arm_right_twist.linear.z = target_arm_right_twist.linear.z
                    control_arm_right_twist.angular.x = target_arm_right_twist.angular.x
                    control_arm_right_twist.angular.y = target_arm_right_twist.angular.y
                    control_arm_right_twist.angular.z = target_arm_right_twist.angular.z
            elif element_add(target_arm_right_twist) < element_add(control_arm_right_twist):
                if element_add(target_arm_right_twist) > element_add(control_arm_right_twist) - 0.02:
                    control_arm_right_twist.linear.x = target_arm_right_twist.linear.x
                    control_arm_right_twist.linear.y = target_arm_right_twist.linear.y
                    control_arm_right_twist.linear.z = target_arm_right_twist.linear.z
                    control_arm_right_twist.angular.x = target_arm_right_twist.angular.x
                    control_arm_right_twist.angular.y = target_arm_right_twist.angular.y
                    control_arm_right_twist.angular.z = target_arm_right_twist.angular.z
                else:
                    if(arm_right_S.linear.x != 0):
                        control_arm_right_twist.linear.x = control_arm_right_twist.linear.x - 0.02
                    if(arm_right_S.linear.y != 0):
                        control_arm_right_twist.linear.y = control_arm_right_twist.linear.y - 0.02
                    if(arm_right_S.linear.z != 0):
                        control_arm_right_twist.linear.z = control_arm_right_twist.linear.z - 0.02    
                    if(arm_right_S.angular.x != 0):
                        control_arm_right_twist.angular.x = control_arm_right_twist.angular.x - 0.02
                    if(arm_right_S.angular.y != 0):
                        control_arm_right_twist.angular.y = control_arm_right_twist.angular.y - 0.02
                    if(arm_right_S.angular.z != 0):
                        control_arm_right_twist.angular.z = control_arm_right_twist.angular.z - 0.02 
            else:
                    control_arm_right_twist.linear.x = target_arm_right_twist.linear.x
                    control_arm_right_twist.linear.y = target_arm_right_twist.linear.y
                    control_arm_right_twist.linear.z = target_arm_right_twist.linear.z
                    control_arm_right_twist.angular.x = target_arm_right_twist.angular.x
                    control_arm_right_twist.angular.y = target_arm_right_twist.angular.y
                    control_arm_right_twist.angular.z = target_arm_right_twist.angular.z

            if target_speed_gripper > control_speed_gripper:
                control_speed_gripper = min( target_speed_gripper, control_speed_gripper + 0.02 )
            elif target_speed_gripper < control_speed_gripper:
                control_speed_gripper = max( target_speed_gripper, control_speed_gripper - 0.02 )
            else:
                control_speed_gripper = target_speed_gripper       
 
            eod_robot_twist_pub = Eod_robot_twist()

            eod_robot_twist_pub.vecihle_linear.x = control_speed_vecihle
            eod_robot_twist_pub.vecihle_linear.y = 0
            eod_robot_twist_pub.vecihle_linear.z = 0
            eod_robot_twist_pub.vecihle_angular.x = 0
            eod_robot_twist_pub.vecihle_angular.y = 0
            eod_robot_twist_pub.vecihle_angular.z = control_turn_vecihle

            eod_robot_twist_pub.arm_left_linear.x = control_arm_left_twist.linear.x
            eod_robot_twist_pub.arm_left_linear.y = control_arm_left_twist.linear.y
            eod_robot_twist_pub.arm_left_linear.z = control_arm_left_twist.linear.z
            eod_robot_twist_pub.arm_left_angular.x = control_arm_left_twist.angular.x
            eod_robot_twist_pub.arm_left_angular.y = control_arm_left_twist.angular.y
            eod_robot_twist_pub.arm_left_angular.z = control_arm_left_twist.angular.z

            eod_robot_twist_pub.arm_right_linear.x = control_arm_right_twist.linear.x
            eod_robot_twist_pub.arm_right_linear.y = control_arm_right_twist.linear.y
            eod_robot_twist_pub.arm_right_linear.z = control_arm_right_twist.linear.z
            eod_robot_twist_pub.arm_right_angular.x = control_arm_right_twist.angular.x
            eod_robot_twist_pub.arm_right_angular.y = control_arm_right_twist.angular.y
            eod_robot_twist_pub.arm_right_angular.z = control_arm_right_twist.angular.z

            eod_robot_twist_pub.gripper_velocity.data = control_speed_gripper

            eod_robot_twist_pub.arm_left_joint_or_task.data = arm_left_joint_or_task

            eod_robot_twist_pub.arm_right_joint_or_task.data = arm_right_joint_or_task

            pub.publish(eod_robot_twist_pub)

    except Exception as e:
        print(e)

    finally:
        eod_robot_twist_pub = Eod_robot_twist()
        eod_robot_twist_pub.vecihle_linear.x = 0; eod_robot_twist_pub.vecihle_linear.y = 0; eod_robot_twist_pub.vecihle_linear.z = 0
        eod_robot_twist_pub.vecihle_angular.x = 0; eod_robot_twist_pub.vecihle_angular.y = 0; eod_robot_twist_pub.vecihle_angular.z = 0
        eod_robot_twist_pub.arm_left_linear.x = 0; eod_robot_twist_pub.arm_left_linear.y = 0; eod_robot_twist_pub.arm_left_linear.z = 0
        eod_robot_twist_pub.arm_left_angular.x = 0; eod_robot_twist_pub.arm_left_angular.y = 0; eod_robot_twist_pub.arm_left_angular.z = 0
        eod_robot_twist_pub.arm_right_linear.x = 0; eod_robot_twist_pub.arm_right_linear.y = 0; eod_robot_twist_pub.arm_right_linear.z = 0
        eod_robot_twist_pub.arm_right_angular.x = 0; eod_robot_twist_pub.arm_right_angular.y = 0; eod_robot_twist_pub.arm_right_angular.z = 0
        eod_robot_twist_pub.gripper_velocity.data = 0
        eod_robot_twist_pub.arm_left_joint_or_task.data = False
        eod_robot_twist_pub.arm_right_joint_or_task.data = False

        pub.publish(eod_robot_twist_pub)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)