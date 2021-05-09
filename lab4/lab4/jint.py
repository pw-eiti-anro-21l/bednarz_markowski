import sys
import time
import math
import threading
from lab4_srv.srv import Jint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class Limit: #prosta klasa do przechowania limitow jointow
    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper

class JintService(Node):

    def __init__(self):
        super().__init__('JINT_Service')
        qos_profile = QoSProfile(depth=10)
        self.srv = self.create_service(Jint, 'jint_control_srv', self.jint_control_srv_callback)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.joint_state = JointState()

        self.names = ["poloczenie-baza-czlon1", "poloczenie-czlon1-czlon2", "poloczenie-czlon2-czlon3"]

        #pozycja startowa
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.0
        self.currentJ1 = self.j1
        self.currentJ2 = self.j2
        self.currentJ3 = self.j3

        self.j1Limit = Limit(-3.14, 3.14)
        self.j2Limit = Limit(-1.57, 0.0)
        self.j3Limit = Limit(-1.57, 1.57)

        self.result=""

        publishingThread = threading.Thread(target=self.publishNewStates)
        publishingThread.start()
        
    def jint_control_srv_callback(self, request, response):

        if request.time <= 0:
            response.output = "Invalid interpolation time; Interpolation impossible; Terminating"
            return response

        if request.type != "Linear" and request.type != "Spline":
            response.output = "Invalid interpolation type; Avaible types: Linear, Spline; Terminating"
            return response

        
        self.currentJ1 = self.j1
        self.currentJ2 = self.j2
        self.currentJ3 = self.j3
        targetJ1 = request.j1pose
        targetJ2 = request.j2pose
        targetJ3 = request.j3pose
        targetTime = request.time
    	
        T = 0.1
        steps = int(request.time/T)
        for k in range(1, steps+1):
            if request.type == "Linear":
                self.currentJ1 = self.interpolateLin(self.j1, targetJ1, targetTime, k*T)
                self.currentJ2 = self.interpolateLin(self.j2, targetJ2, targetTime, k*T)
                self.currentJ3 = self.interpolateLin(self.j3, targetJ3, targetTime, k*T)

            if request.type == "Spline":
                self.currentJ1 = self.interpolateSpline(self.j1, targetJ1, targetTime, k*T)
                self.currentJ2 = self.interpolateSpline(self.j2, targetJ2, targetTime, k*T)
                self.currentJ3 = self.interpolateSpline(self.j3, targetJ3, targetTime, k*T)
            time.sleep(0.1)

        self.j1 = self.currentJ1
        self.j2 = self.currentJ2
        self.j3 = self.currentJ3
        response.output= "Interpolation ( " + request.type + " ) succesful! Absolute errors: " + \
        str(abs(targetJ1 - self.j1)) + ", " + str(abs(targetJ2 - self.j2)) + ", " + str(abs(targetJ3 - self.j3))
        return response

    def publishNewStates(self):
        while True:
            try:
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = self.names
                self.joint_state.position = [self.currentJ1, self.currentJ2, self.currentJ3]
                # update transform
                # (moving in a circle with radius=2)
                self.odom_trans.header.stamp = now.to_msg()
                # send the joint state and transform
                self.publisher.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)
                time.sleep(0.1)
            except KeyboardInterrupt:
                exit(0)

    def interpolateLin(self, x0, x1, t, timePassed): 
        return x0 + (x1-x0)/t*timePassed

    def interpolateSpline(self, x0, x1, t, timePassed):
        #wzor z wikipedii, t0 = 0
        tx = timePassed/t
        k1 = 0 
        k2 = 0 # pochodne sa zerowe
        a = k1*t - (x1-x0)
        b = -k2*t + (x1-x0)
        qx = (1-tx)*x0 + tx*x1 + tx*(1-tx)*((1-tx)*a+tx*b) 
        return qx


def main(args=None):
    rclpy.init(args=args)

    service = JintService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
