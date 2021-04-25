import math
import mathutils as m
from PyKDL import *
import rclpy
import sys
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
import time
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory



class nonKdlNode(Node):
    def __init__(self):
    	super().__init__("kdl_dkin")
    	self.publisher_ = self.create_publisher(PoseStamped, '/kdl_pose', QoSProfile(depth=10))
    	self.get_logger().info("kdl has been started")
    	self.subscription = self.create_subscription(JointState, 'joint_states', self.forwardKin,10)

    	
    def forwardKin(self,msg):
    	
    	# wczytanie danych z pliku
    	DHtab = []
    	plik = "DHtab.txt"
    	
    	f = open(os.path.join(get_package_share_directory('lab3'),plik),"r")
    	for line in f:
    		if line[0] == 'i':
    			continue
    		DHtab.append(line.split())
    	f.close()
    	
    	chain = Chain()
    	frame = Frame()
    	
    	frame0 = frame.DH(0, 0, 0, 0)
    	#joint0 = Joint(Joint.None)
    	#chain.addSegment(Segment(joint0, frame0))
    	
    	a, alfa, d, theta = float(DHtab[0][1]),float(DHtab[0][2]),float(DHtab[0][3]),float(DHtab[0][4])
    	frame1 = frame.DH(a, alfa, d, theta)
    	joint1 = Joint(Joint.RotZ)
    	chain.addSegment(Segment(joint1, frame1))
    	
    	a, alfa, d, theta = float(DHtab[1][1]),float(DHtab[1][2]),float(DHtab[1][3]),float(DHtab[1][4])
    	frame2 = frame.DH(a, alfa, d, theta)
    	joint2 = Joint(Joint.RotZ)
    	chain.addSegment(Segment(joint2, frame2))
    	
    	a, alfa, d, theta = float(DHtab[2][1]),float(DHtab[2][2]),float(DHtab[2][3]),float(DHtab[2][4])
    	frame3 = frame.DH(a, alfa, d, theta)
    	joint3 = Joint(Joint.RotZ)
    	chain.addSegment(Segment(joint3, frame3))
    	
    	jntarray = JntArray(3)
    	jntarray[0] = msg.position[0]
    	jntarray[1] = msg.position[1]
    	jntarray[2] = -msg.position[2]
    	
    	fksolver = ChainFkSolverPos_recursive(chain)
    	endFrame = Frame()
    	fksolver.JntToCart(jntarray, endFrame)
    	
    	xyz = endFrame.p
    	quater = endFrame.M.GetQuaternion()
    	
    	pose = PoseStamped()
    	pose.header.stamp = ROSClock().now().to_msg()
    	pose.header.frame_id = "baza"
    	pose.pose.position.x = float(xyz[0])
    	pose.pose.position.y = float(xyz[1])
    	pose.pose.position.z = float(xyz[2])
    	pose.pose.orientation = Quaternion(x=quater[0], y=quater[1], z=quater[2], w=quater[3])
    	
    	self.publisher_.publish(pose)
    	

def main(args=None):
    if args is None:
    	args = sys.argv
    rclpy.init(args=args)
    node = nonKdlNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
