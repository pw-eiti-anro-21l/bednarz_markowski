import sys
import time
from lab4_srv.srv import Oint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion, PoseStamped


class OintService(Node):

    def __init__(self):
        super().__init__('OINT_Service')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.oint_control_srv_callback)
        self.publisher = self.create_publisher(PoseStamped, "/pose", QoSProfile(depth=10))
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "baza"
        #pozycja startowa
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        

    def oint_control_srv_callback(self, request, response):
    	T = 0.1
    	steps = int(request.time/T)
    	
    	if request.type == "Linear":
    		for k in range(1, steps):
    			now = self.get_clock().now()
    			self.pose_stamped.position.x = self.x + (request.x-self.x)/steps*k
    			self.pose_stamped.position.y = self.y + (request.y-self.y)/steps*k
    			self.pose_stamped.position.z = self.z + (request.x-self.z)/steps*k
    			
    			current_roll = self.roll +(request.roll-self.roll)/steps*k
    			current_pitch = self.pitch +(request.pitch-self.pitch)/steps*k
    			current_yaw = self.yaw +(request.yaw-self.yaw)/steps*k
    			self.pose_stamped.pose.orientation = self.euler_to_quaternion(current_roll, current_pitch, current_yaw) 
    			self.publisher.publish(self.pose_stamped)
    			time.sleep(T)
    	return response
   
    def euler_to_quaternion(roll, pitch, yaw):
    	qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    	qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    	qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    	qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    	return Quaternion(x=qx, y=qy, z=qz, w=qw)



def main(args=None):
    rclpy.init(args=args)

    service = OintService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
