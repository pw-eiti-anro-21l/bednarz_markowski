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
    	super().__init__("nonkdl_dkin")
    	self.publisher_ = self.create_publisher(PoseStamped, '/no_kdl_pose', QoSProfile(depth=10))
    	self.get_logger().info("nonkdl has been started")
    	self.subscription = self.create_subscription(JointState, 'joint_states', self.forwardKin,10)
    	
    	
    def forwardKin(self,msg):
    	
    	M = self.wczytaj()
    	
    	Mk = M[0] @ M[1] @ M[2]
    
    
    	xyz = Mk.to_translation()
    	rpy = Mk.to_euler()
    	quater = rpy.to_quaternion()
    	
    
    	pose = PoseStamped()
    	pose.header.stamp = ROSClock().now().to_msg()
    	pose.header.frame_id = "baza"
    	
    	pose.pose.position.x = xyz[0]
    	pose.pose.position.y = xyz[1]
    	pose.pose.position.z = xyz[2]
    	pose.pose.orientation = Quaternion(w=quater[0], x=quater[1], y=quater[2], z=quater[3])
    	
    	self.publisher_.publish(pose)
    	
    def wczytaj(self):
   	# wczytanie danych z pliku
    	DHtab = []
    	M = []
    	plik = "DHtab.txt"
    	
    	f = open(os.path.join(get_package_share_directory('lab3'),plik),"r")
    	for line in f:
    		DHtab.append(line.split())
    	f.close()

    	for linia in DHtab:
    		if linia[0] == 'i':
    			continue
    		#stworzenie link
    		a = float(linia[1])
    		alfa = float(linia[2])
    		d = float(linia[3])
    		theta = float(linia[4])
    		nazwa = linia[5]
    		#liczenie macierzy transformacji jednorodnej
    		rotTheta = m.Matrix.Rotation(theta, 4, 'Z')
    		transZ = m.Matrix.Translation((0,0,d))
    		transX = m.Matrix.Translation((a,0,0))
    		rotAlfa = m.Matrix.Rotation(alfa, 4, 'X')
    		
    		T = rotAlfa @ transX @ rotTheta @ transZ
    		M.append(T)
    	return M
    	
    	

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
