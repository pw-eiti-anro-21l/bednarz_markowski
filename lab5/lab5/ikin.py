import math
import mathutils as m
import rclpy
import sys
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class Ikin(Node):
    def __init__(self):
        super().__init__("IKIN_node")
        qos_profile = QoSProfile(depth=10)
        self.joint_pub= self.create_publisher(JointState, 'joint_states', QoSProfile(depth=10))
        self.get_logger().info("IKIN has been started")
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.listener_pose, QoSProfile(depth=10))
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.joint_state = JointState()
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.markerMAX = 1000
        self.markerCount = 0
        self.markerArray = MarkerArray()
        self.marker_pub = self.create_publisher(MarkerArray, "/path", QoSProfile(depth=10))
        self.marker = Marker()
        self.marker.id  = 0
        self.marker.action = Marker.DELETEALL
        self.marker.header.frame_id = "odom"
        self.marker.header.stamp

        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 0.5
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.id = 0
    	
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
        #self.a2 = 0.3 #nie ma tego w dh
        self.a2 = float(DHtab[3][1])
        self.names = ["poloczenie-baza-czlon1", "poloczenie-czlon1-czlon2", "poloczenie-czlon2-czlon3"]
        self.currentJ1 = 0.0
        self.currentJ2 = 0.0
        self.currentJ3 = 0.0
        self.get_logger().info("test")
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0 
        self.err = False

    def inverted_kin(self):
        self.err = False
        # obliczenie odwrotnej kinematyki
        warunek1 = bool(math.sqrt(self.x **2 + self.y**2 +(self.z - self.d1)**2)<= self.a1 + self.a2)
        warunek2 = bool(math.sqrt(self.x **2 + self.y**2 +(self.z - self.d1)**2)>= self.a1 - self.a2)
        if warunek1 and warunek2:
            #jezeli zadana pozycja jest w zasiegu manipulatora
            #wyznaczenie theta1
            newTheta1 = self.currentJ1
            if self.x>=0 and self.y >=0:
                newTheta1 = math.atan(self.y/self.x)
            if self.x< 0 and self.y >=0:
                newTheta1 = math.atan(self.y/self.x) + math.pi
            if self.x< 0 and self.y <0:
                newTheta1 = math.atan(self.y/self.x) - math.pi
            if self.x>=0 and self.y <0:
                newTheta1 = math.atan(self.y/self.x)
            #sprawdzenie ograniczenia jointu
            if newTheta1 > 3.14 or newTheta1 < -3.14:
                self.err = True
                self.get_logger().info("Wyznaczenie pozycji jest niemozliwe, przekroczono limit joint 1")
            else:
                self.currentJ1 = newTheta1

            #wyznaczenie theta3
            newTheta3 = self.currentJ3
            newTheta3 = math.acos((self.x**2 + self.y**2 + (self.z-self.d1)**2 - self.a1 **2 - self.a2**2)/(2*self.a1*self.a2))
            #sprawdzenie limitow
            if newTheta3 > 1.57 or newTheta3 < -1.57:
                self.err = True
                self.get_logger().info("Wyznaczenie pozycji jest niemozliwe, przekroczono limit joint 3")
            else:
                self.currentJ3 = newTheta3
            

            #wyznaczenie theta2
            newTheta2 = self.currentJ2
            newTheta2 = -math.atan((self.z-self.d1)/(math.sqrt(self.x**2 + self.y**2)))
            newTheta2 -= math.asin(self.a2*math.sin(self.currentJ3)/math.sqrt(self.x **2 + self.y**2 +(self.z - self.d1)**2))
            #sprawdzenie limitow
            if newTheta2 > 0 or newTheta2 < -1.57:
                self.err = True
                self.get_logger().info("Wyznaczenie pozycji jest niemozliwe, przekroczono limit joint 2")
            else:
                self.currentJ2 = newTheta2

        else:
            self.err = True
            self.get_logger().info("Wyznaczenie pozycji jest niemozliwe, zadana pozycja poza zasiegiem manipulatora")
    
    def listener_pose(self,msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.inverted_kin()

        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = self.names
        self.joint_state.position = [self.currentJ1, self.currentJ2, self.currentJ3]
        #update transform
        self.odom_trans.header.stamp = now.to_msg()
        #send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

        #zmiana koloru markera w zaleznosci od tego czy udalo sie obliczyc nowe pozycje
        if self.err:
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0   
        else:
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0

        self.marker.pose.position.x = self.x
        self.marker.pose.position.y = self.y
        self.marker.pose.position.z = self.z
        if self.markerCount > self.markerMAX:
            self.markerArray.markers.pop(0)
        self.markerCount += 1
        self.markerArray.markers.append(self.marker)
        for m in self.markerArray.markers:
            m.id = self.id
            self.id+=1
        self.marker_pub.publish(self.markerArray)
    	

 
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
