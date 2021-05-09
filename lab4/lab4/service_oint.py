import sys
import time
from lab4_srv.srv import Oint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion, PoseStamped
from math import sin, cos
import threading


class OintService(Node):

    def __init__(self):
        super().__init__('OINT_Service')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.oint_control_srv_callback)
        self.publisher = self.create_publisher(PoseStamped, "/pose", QoSProfile(depth=10))
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "odom"
        #pozycja startowa
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        publishingThread = threading.Thread(target=self.publishNewStates)
        publishingThread.start()
        

    def oint_control_srv_callback(self, request, response):
    	T = 0.1
    	
    	if request.type == "Linear":
    		steps = int(request.time/T)
    		delta_x = (request.x-self.x)/steps
    		delta_y = (request.y-self.y)/steps
    		delta_z = (request.z-self.z)/steps
    		delta_roll = (request.roll-self.roll)/steps
    		delta_pitch = (request.pitch-self.pitch)/steps
    		delta_yaw = (request.yaw-self.yaw)/steps
    	
    		for k in range(1, steps+1):
    			now = self.get_clock().now()
    			self.pose_stamped.header.stamp = now.to_msg()
    			self.x += delta_x
    			self.pose_stamped.pose.position.x = self.x
    			self.y += delta_y
    			self.pose_stamped.pose.position.y = self.y
    			self.z += delta_z
    			self.pose_stamped.pose.position.z = self.z
    			self.roll += delta_roll
    			self.pitch += delta_pitch
    			self.yaw += delta_yaw
    			
    			self.pose_stamped.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw) 
    			self.publisher.publish(self.pose_stamped)
    			time.sleep(T)

    		return response
    		
    def publishNewStates(self):
    	while True:
    		try:
	    		now = self.get_clock().now()
	    		self.pose_stamped.pose.position.x = self.x
	    		self.pose_stamped.pose.position.y = self.y
	    		self.pose_stamped.pose.position.z = self.z
	    		self.pose_stamped.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw) 
	    		self.pose_stamped.header.stamp = now.to_msg()
	    		self.pose_stamped.header.frame_id = 'odom'
	    		self.publisher.publish(self.pose_stamped)
	    		time.sleep(0.1)
	    	except KeyboardInterrupt:
	    		exit(0)
			
    def euler_to_quaternion(self, roll, pitch, yaw):
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
