import sys
import time
from lab4_srv.srv import Oint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from math import sin, cos, pi
import threading
from lab4_srv.srv import Figure

class OintService(Node):

    def __init__(self):
        super().__init__('OINT_Service')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.oint_control_srv_callback)
        self.figure_srv = self.create_service(Figure, 'oint_figure_srv', self.oint_figure_srv_callback)
        self.publisher = self.create_publisher(PoseStamped, "/pose", QoSProfile(depth=10))
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "baza"
        #self.marker_pub = self.create_publisher(Marker, "/path", QoSProfile(depth=10))
        #pozycja startowa
        self.x = 1.7
        self.y = 0.0
        self.z = 0.5
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        publishingThread = threading.Thread(target=self.publishNewStates)
        publishingThread.start()
        
        #self.marker = Marker()
        #self.marker.id  = 0
        #self.marker.action = Marker.DELETEALL
        #self.marker.header.frame_id = "odom"
        #self.marker.header.stamp

        #self.marker.type = Marker.LINE_STRIP
        #self.marker.action = Marker.ADD
        #self.marker.scale.x = 0.1
        #self.marker.scale.y = 0.1
        #self.marker.scale.z = 0.1
        #self.marker.color.a = 1.0
        #self.marker.color.r = 1.0
        #self.marker.color.g = 1.0
        #self.marker.color.b = 0.0
        
        

    def oint_control_srv_callback(self, request, response):
    	T = 0.1
    	steps = int(request.time/T)
    	if request.time <= 0:
    		response.output = "Invalid interpolation time; Interpolation impossible; Terminating"
    		return response
    	if request.type != "Linear" and request.type != "Spline":
    		response.output = "Invalid interpolation type; Avaible types: Linear, Spline; Terminating"
    		return response

    	
    	if request.type == "Linear":
    		
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
    			time.sleep(T)
    			#point = Point()
    			#point.x = self.x
    			#point.y = self.y
    			#point.z = self.z
    			#self.marker.points.append(point)
    			#self.marker_pub.publish(self.marker)
    			
    		result = "Interpolation 'Linear' succesful!"
    		response.output = result
    		return response
    		
    	elif request.type == "Spline":
    		start_x = self.x
    		start_y = self.y
    		start_roll = self.roll
    		start_pitch = self.pitch
    		start_z = self.z
    		start_yaw = self.yaw
    		for k in range(1, steps+1):
    			now = self.get_clock().now()
    			self.pose_stamped.header.stamp = now.to_msg()
    			self.x = self.interpolateSpline(start_x, request.x, request.time, k*T)
    			self.pose_stamped.pose.position.x = self.x
    			self.y = self.interpolateSpline(start_y, request.y, request.time, k*T)
    			self.pose_stamped.pose.position.y = self.y
    			self.z = self.interpolateSpline(start_z, request.z, request.time, k*T)
    			self.pose_stamped.pose.position.z = self.z
    			self.roll = self.interpolateSpline(start_roll, request.roll, request.time, k*T)
    			self.pitch = self.interpolateSpline(start_pitch, request.pitch, request.time, k*T)
    			self.yaw = self.interpolateSpline(start_yaw, request.yaw, request.time, k*T)
    			
    			self.pose_stamped.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw) 
    			time.sleep(T)
    			#point = Point()
    			#point.x = self.x
    			#point.y = self.y
    			#point.z = self.z
    			#self.marker.points.append(point)
    			#self.marker_pub.publish(self.marker)
    		result = "Interpolation 'Spline' succesful!"
    		response.output = result
    		return response
    		
    def oint_figure_srv_callback(self, request, response):
    	T = 0.1
    	steps = int(request.time/T)
    	start_y = self.y
    	start_z = self.z
    	if request.time <= 0:
    		response.output = "Invalid interpolation time; Interpolation impossible; Terminating"
    		return response
    	if request.figure != "Ellipse" and request.figure != "Rectangle":
    		response.output = "Invalid interpolation type; Avaible types: Ellipse, Rectangle; Terminating"
    		return response
    	if(request.figure == 'Rectangle'):
    		obw = 2*request.param_a +2*request.param_b
    		
    		j = int((request.param_a/obw)*steps)
    		k = int((request.param_b/obw)*steps)
    		for i in range(1,steps+1):
	    		if i <= j:
	    			self.y = self.y + request.param_a/(request.time*j/steps)*T
	    			time.sleep(0.1)
	    		if i > j and i <= j + k:
	    			self.z  = self.z + request.param_b/(request.time*k/steps)*T
	    			time.sleep(0.1)
	    		if i > j+k and i <= 2*j + k:
	    			self.y = self.y - request.param_a/(request.time*j/steps)*T
	    			time.sleep(0.1)
	    		if i > 2*j +k and i <= 2*(j+k):
	    			self.z = self.z - request.param_b/(request.time*k/steps)*T
	    			time.sleep(0.1)
	    			
	    	result = "Figure 'Rectangle': succesful!"
    		response.output = result
    		return response
    	if(request.figure == 'Ellipse'):
    		for i in range(1,steps+1):
    			self.y = start_y + request.param_a*cos(2*pi*T/request.time*i) - request.param_a
    			self.z = start_z + request.param_b*sin(2*pi*T/request.time*i)
    			time.sleep(0.1)
    		result = "Figure 'Ellipse': succesful!"
    		response.output = result
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
	    		self.pose_stamped.header.frame_id = 'baza'
	    		self.publisher.publish(self.pose_stamped)
	    		time.sleep(0.1)
	    	except KeyboardInterrupt:
	    		exit(0)
	    		
    def interpolateSpline(self, x0, x1, t, timePassed):
        #wzor z wikipedii, t0 = 0
        tx = timePassed/t
        k1 = 0 
        k2 = 0 # pochodne sa zerowe
        a = k1*t - (x1-x0)
        b = -k2*t + (x1-x0)
        qx = (1-tx)*x0 + tx*x1 + tx*(1-tx)*((1-tx)*a+tx*b) 
        return qx
			
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
