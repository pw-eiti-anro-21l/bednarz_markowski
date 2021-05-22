import math
import mathutils as m
import rclpy
import sys
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory



class Ikin(Node):
    def __init__(self):
    	super().__init__("IKIN_node")
    	self.joint_pub= self.create_publisher(JointState, 'ikdl_joint', QoSProfile(depth=10))
    	self.get_logger().info("IKIN has been started")
    	self.pose_sub = self.create_subscription(PoseStamped, 'oint_pose', self.listener_pose, QoSProfile(depth=10))
    	
    	# wczytanie danych z pliku
    	DHtab = []
    	plik = "DHtab.txt"
    	
    	f = open(os.path.join(get_package_share_directory('lab5'),plik),"r")
    	for line in f:
    		if line[0] == 'i':
    			continue
    		DHtab.append(line.split())
    	f.close()
    	self.d1 = float(DHtab[0][3])
    	self.alfa2 = float(DHtab[1][2])
    	self.a3 = float(DHtab[2][1])
    	
    def inverted_kin(self):
    
    	joint_state = JoitState()
    	now = self.get_clock().now()
    	joint_state.header.stamp = now.to_msg()
    	self.joint_state.name = ["czlon1", "czlon2", "czlon3"]
    	
    	# wyliczenie odwrotnej kinematyki
    	
    	self.get_logger().error("Wyznaczenie pozycji jest niemozliwe")
    
    def listener_pose(self,msg):
    	self.x = msg.pose.position.x
    	self.y = msg.pose.position.y
    	self.z = msg.pose.position.z
    	
    	self.inverted_kin()
    	

 
def main(args=None):
    if args is None:
    	args = sys.argv
    rclpy.init(args=args)
    node = Ikin()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
