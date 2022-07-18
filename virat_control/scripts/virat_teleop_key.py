#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty


msg = """
Controls for Virat
============================================
                    |                      |
Move around using:  |  I to increase speed |  
                    |  U to decrease speed |
        W           |                      |
     A  S  D        |                      |
                    | CTRL-C to quit       |
============================================   
"""

moveCmd = { 'w':(10, 0),  'a':(0,-10),
            's':(-10, 0), 'd':(0, 10)
          }
          
incCmd = {'i':0.10, 'u':0.10}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1) #read 1 byte
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
 
settings = termios.tcgetattr(sys.stdin)
    
rospy.init_node('virat_teleop')
pub = rospy.Publisher('virat/cmd_vel', Twist, queue_size=3)
lin_x = 0
ang_z = 0
    
try:
    print(msg)
    speed_multiplier = 1    
    while(1):
        key = getKey()
        if key in moveCmd.keys():
            lin_x = moveCmd[key][0]*speed_multiplier
            ang_z = moveCmd[key][1]*speed_multiplier
        elif key == '':
            lin_x = 0
            ang_z = 0
        elif key == 'i':
            speed_multiplier *= 1+incCmd[key]
            speed_multiplier = min(2,speed_multiplier)  # upper speed limit
            print(f"Moving at {100*speed_multiplier}% speed")
        elif key == 'u':
            speed_multiplier *= 1-incCmd[key]
            speed_multiplier = max(0.5,speed_multiplier)  # lower speed limit 
            print(f"Moving at {100*speed_multiplier}% speed")  
        elif key == '\x03':
            break
        
        twist = Twist()
        twist.linear.x = lin_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang_z
        
        
        pub.publish(twist)
        
except Exception as e:
    print(e)
        
finally:
    twist = Twist()
    twist.linear.x = lin_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ang_z
        
    pub.publish(twist)
    
        
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)       	        
