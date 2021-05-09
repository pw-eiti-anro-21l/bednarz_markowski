import sys
import time
from lab4_srv.srv import Jint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped



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
        
    def jint_control_srv_callback(self, request, response):
        T = 0.1
        steps = int(request.time/T)
        currentJ1 = self.j1
        currentJ2 = self.j2
        currentJ3 = self.j3
    	
        if request.type == "Linear":
            for k in range(1, steps):
                now = self.get_clock().now()
                currentJ1 = self.j1 + (request.j1pose-self.j1)/(request.time) *k*T
                currentJ2 = self.j2 + (request.j2pose-self.j2)/(request.time) *k*T
                currentJ3 = self.j3 + (request.j3pose-self.j3)/(request.time) *k*T
                #currentJ1 = self.interpolateLin(self.j1, request.j1pose, steps, k)
                #currentJ2 = self.interpolateLin(self.j2, request.j2pose, steps, k)
                #currentJ3 = self.interpolateLin(self.j3, request.j3pose, steps, k)
    			

                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = self.names
                self.joint_state.position = [currentJ1, currentJ2, currentJ3]
                  # update transform
                 # (moving in a circle with radius=2)
                self.odom_trans.header.stamp = now.to_msg()

                 # send the joint state and transform
                self.publisher.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)
                time.sleep(T)
            self.j1 = currentJ1
            self.j2 = currentJ2
            self.j3 = currentJ3
        return response

    def interpolateLin(x0, x1, step, t): #t - chwila czasu
        return x0 + (x1-x0)/step*t



def main(args=None):
    rclpy.init(args=args)

    service = JintService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
