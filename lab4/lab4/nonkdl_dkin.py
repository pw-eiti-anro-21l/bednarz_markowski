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
    	self.publisher_ = self.create_publisher(PoseStamped, '/non_kdl_pose', QoSProfile(depth=10))
    	self.get_logger().info("nonkdl has been started")
    	self.subscription = self.create_subscription(JointState, 'joint_states', self.forwardKin,10)

    	
    def forwardKin(self,msg):
    	# sprawdzenie danych czy poza limitem:
    	if msg.position[0] <-3.14 or msg.position[0] > 3.14:
    		self.get_logger().info("przekroczono limit baza-czlon1")
    		return
    	if msg.position[1] <-1.57 or msg.position[1] > 0:
    		self.get_logger().info("przekroczono limit czlon1-czlon2")
    		return
    	if msg.position[2] <-1.57 or msg.position[2] > 1.57:
    		self.get_logger().info("przekroczono limit czlon2-czlon3")
    		return
    	
    	
    	# wczytanie danych z pliku
    	DHtab = []
    	plik = "DHtab.txt"
    	
    	f = open(os.path.join(get_package_share_directory('lab3'),plik),"r")
    	for line in f:
    		if line[0] == 'i':
    			continue
    		DHtab.append(line.split())
    	f.close()

    	a, alfa, d, theta = float(DHtab[0][1]),float(DHtab[0][2]),float(DHtab[0][3]),float(DHtab[0][4])
    	#liczenie macierzy transformacji jednorodnej
    	rotTheta = m.Matrix.Rotation(theta+msg.position[0], 4, 'Z')
    	transZ = m.Matrix.Translation((0,0,d))
    	transX = m.Matrix.Translation((a,0,0))
    	rotAlfa = m.Matrix.Rotation(alfa, 4, 'X')
    	M1 = rotAlfa @ transX @ rotTheta @ transZ
    	
    	a, alfa, d, theta = float(DHtab[1][1]),float(DHtab[1][2]),float(DHtab[1][3]),float(DHtab[1][4])
    	#liczenie macierzy transformacji jednorodnej
    	rotTheta = m.Matrix.Rotation(theta+msg.position[1], 4, 'Z')
    	transZ = m.Matrix.Translation((0,0,d))
    	transX = m.Matrix.Translation((a,0,0))
    	rotAlfa = m.Matrix.Rotation(alfa, 4, 'X')
    	M2 = rotAlfa @ transX @ rotTheta @ transZ
    	
    	a, alfa, d, theta = float(DHtab[2][1]),float(DHtab[2][2]),float(DHtab[2][3]),float(DHtab[2][4])
    	#liczenie macierzy transformacji jednorodnej
    	rotTheta = m.Matrix.Rotation(theta+msg.position[2], 4, 'Z')
    	transZ = m.Matrix.Translation((0,0,d))
    	transX = m.Matrix.Translation((a,0,0))
    	rotAlfa = m.Matrix.Rotation(alfa, 4, 'X')
    	M3 = rotAlfa @ transX @ rotTheta @ transZ
    	
    	rotTheta = m.Matrix.Rotation(0, 4, 'Z')
    	transZ = m.Matrix.Translation((0,0,0))
    	transX = m.Matrix.Translation((0.3,0,0))
    	rotAlfa = m.Matrix.Rotation(0, 4, 'X')
    	M4 = rotAlfa @ transX @ rotTheta @ transZ

    	Mk = M1 @ M2 @ M3 @ M4
    
    
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
