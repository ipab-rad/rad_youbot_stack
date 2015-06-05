#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# adapted by Alexandre Silva for KUKA youbot teleop arm control

import rospy, math, time, copy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty, signal

msg = """
Reading from the keyboard to actuate individual joints
---------------------------
Initial configuration is 0 rads at each joint

To actuate specific joint pass the joint index, for example:
1

This increases the desired configuration by 0.5 rads for that particular joint

To switch between increase/decreasing joint angles use:
d for decreasing
i for increasing


CTRL-C to quit
"""

jointBindings = [
		'1', 	# joint1
		'2',	# joint2
		'3', 	# joint3
		'4', 	# joint4
	    '5'  ]

moveBindings = [
		'd', 	# decrease angle
		'i', 	# increase angle
	       ]

class TimeoutException(Exception): 
    pass 

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) #this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
       #print "Read key"
    except TimeoutException:
       #print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

TOPIC_J_CMD = '/arm_1/arm_controller/command'

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	# pub = rospy.Publisher('cmd_vel', Twist)	
	pub = rospy.Publisher(TOPIC_J_CMD, JointTrajectory)
	rospy.init_node('teleop_arm_keyboard')

	dt=2
	dtheta = 0.5
	desired = [0,0,0,0,0]
	mov_direction = 1
	
	try:
		print msg
		
		while(1):
			key = getKey()
			
			if key in jointBindings:
				jt = JointTrajectory()	
				# fill the header
				jt.header.seq = 0
				jt.header.stamp.secs = 0 #secs
				jt.header.stamp.nsecs = 0 #nsecs
				jt.header.frame_id = 'base_link' 
				# specify the joint names
				jt.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
				# joint points
				jtp = JointTrajectoryPoint()
				desired[jointBindings.index(key)] += mov_direction*dtheta
				jtp.positions = desired
				jtp.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
				jtp.accelerations =[0.0, 0.0, 0.0, 0.0, 0.0]
				jtp.time_from_start = rospy.Duration.from_sec(2*(0+1))
				jt.points.append(copy.deepcopy(jtp))						
				pub.publish(jt)
				print 'instructed desired joint configuration:'
				print desired
			elif key in moveBindings:
				# decrease
				if moveBindings.index(key)==0 :
					mov_direction = -1
					print 'now decreasing joint angles'
				else:
					mov_direction = 1
					print 'now increasing joint angles'


	except:
		print "Unexpected error:", sys.exc_info()[0]

	finally:
		jt = JointTrajectory()
		pub.publish(jt)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

