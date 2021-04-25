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
    	self.get_logger().info("nonkdl has been started")
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
"""
    	chain = Chain()
    	a1, d1= float(DHtab[0][1]),float(DHtab[0][3])
    	baza_czlon1 = Joint(Joint.RotZ)
    	frame1 = Frame(Rotation.RPY(0, 0, 0), Vector(a2, 0, d1))
    	
    	a, alfa, d, theta = float(DHtab[1][1]),float(DHtab[1][2]),float(DHtab[1][3]),float(DHtab[1][4])
    	
    	a, alfa, d, theta = float(DHtab[2][1]),float(DHtab[2][2]),float(DHtab[2][3]),float(DHtab[2][4])
    	"""
    	
 
    	
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
