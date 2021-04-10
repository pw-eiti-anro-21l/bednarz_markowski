from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
class StatePublisher(Node):

  def __init__(self):
      rclpy.init()
      super().__init__('state_publisher')
      self.reverse = False

      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))

      self.degree = pi / 180.0
      loop_rate = self.create_rate(30)

      # message declarations
      self.odom_trans = TransformStamped()
      self.odom_trans.header.frame_id = 'odom'
      self.odom_trans.child_frame_id = 'baza'
      self.joint_state = JointState()

      self.names = ["poloczenie-baza-czlon1", "poloczenie-czlon1-czlon2", "poloczenie-czlon2-czlon3"]
      self.states =[0 ,0 ,0]
      self.timer = self.create_timer(0.1, self.timer)

def timer(self):
      try:
            
            for i in range(len(self.states)):
              if self.reverse:
                self.states[i] -= self.degree
              else:
                self.states[i] += self.degree
            
            if self.states[0] >pi/2:
              self.reverse = True
            if self.states[0] < 0:
              self.reverse = False

            # update joint_state
            now = self.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            self.joint_state.name = self.names
            self.joint_state.position = self.states
              # update transform
             # (moving in a circle with radius=2)
            self.odom_trans.header.stamp = now.to_msg()

             # send the joint state and transform
            self.joint_pub.publish(self.joint_state)
            self.broadcaster.sendTransform(self.odom_trans)

            # This will adjust as needed per iteration
           #loop_rate.sleep()

      except KeyboardInterrupt:
          pass

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()
  rclpy.spin(node)

if __name__ == '__main__':
  main()
