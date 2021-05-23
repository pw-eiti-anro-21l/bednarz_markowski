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
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.qos import QoSProfile


class Ikin(Node):
    def __init__(self):
        super().__init__("IKIN_node")
        qos_profile = QoSProfile(depth=10)
        self.joint_pub= self.create_publisher(JointState, 'ikdl_joint', QoSProfile(depth=10))
        self.get_logger().info("IKIN has been started")
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.listener_pose, QoSProfile(depth=10))
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.joint_state = JointState()
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
    	
    	# wczytanie danych z pliku
        DHtab = []
        plik = "DHtab.txt"
    	
        f = open(os.path.join(get_package_share_directory('lab5'),plik),"r")
        for line in f:
            if line[0] == 'i':
                continue
            DHtab.append(line.split())
        f.close()
        self.d1 = float(DHtab[0][3]) # mam nadzieje ze to wysokosc podstawki
        self.alfa2 = float(DHtab[1][2])
        self.a1 = float(DHtab[2][1])
        self.a2 = 0.3 #nie ma tego w dh
        self.names = ["poloczenie-baza-czlon1", "poloczenie-czlon1-czlon2", "poloczenie-czlon2-czlon3"]
        self.currentJ1 = 0.0
        self.currentJ2 = 0.0
        self.currentJ3 = 0.0
        self.get_logger().info("test")
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0 

    def inverted_kin(self):
        # obliczenie odwrotnej kinematyki
        if math.sqrt(self.x **2 + self.y**2 +(self.z - self.d1)**2)< self.a1 + self.a2:
            #jezeli zadana pozycja jest w zasiegu manipulatora
            #wyznaczenie theta1
            if x>=0 and y >=0:
                self.currentJ1 = math.atan(y/x)
            if x< 0 and y >=0:
                self.currentJ1 = math.atan(y/x) + math.pi
            if x< 0 and y <0:
                self.currentJ1 = math.atan(y/x) + math.pi
            if x>=0 and y <0:
                self.currentJ1 = math.atan(y/x)
            #ew mozna tu sprawdzac ograniczenia jointu

            #wyznaczenie theta3
            self.currentJ3 = math.acos((self.x**2 + self.y**2 + (self.z-self.d1)**2 - self.a1 **2 - self.a2**2)/(2*self.a1*self.a2))
            #sprawdzenie limitow

            #wyznaczenie theta2
            self.currentJ2 = math.atan((self.z-self.d1)/(math.sqrt(self.x**2 + self.y**2)))
            self.currentJ2 += math.asin(self.a2*math.sin(self.currentJ3)/math.sqrt(self.x **2 + self.y**2 +(self.z - self.d1)**2))
            #sprawdzenie

        else:
            self.get_logger().info("Wyznaczenie pozycji jest niemozliwe")
    
    def listener_pose(self,msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.get_logger().info("listener dziala")
        self.inverted_kin()
        #now = self.get_clock().now()
        #self.joint_state.header.stamp = now.to_msg()
        #self.joint_state.name = self.names
        #self.joint_state.position = [self.currentJ1, self.currentJ2, self.currentJ3]
        # update transform
        # (moving in a circle with radius=2)
        #self.odom_trans.header.stamp = now.to_msg()
        # send the joint state and transform
        #self.joint_pub.publish(self.joint_state)
        #self.broadcaster.sendTransform(self.odom_trans)
    	

 
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
