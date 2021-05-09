import sys

from lab4_srv.srv import Jint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node


class JintClient(Node):

    def __init__(self):
    	super().__init__('JINT_Client')
    	self.cli = self.create_client(Jint, 'jint_control_srv')
    	while not self.cli.wait_for_service(timeout_sec=2.0):
    		self.get_logger().info('service not available, waiting again...')
    	self.req = Jint.Request()
    	
    def send_request(self):
    
    	try:
    		self.req.j1pose = float(sys.argv[1])
    		self.req.j2pose = float(sys.argv[2])
    		self.req.j3pose = float(sys.argv[3])
    		self.req.time = float(sys.argv[4])
    		self.req.type = sys.argv[5]
    	except ValueError:
    		self.get_logger().info('ValueError')
    	self.future = self.cli.call_async(self.req)
	
def main(args=None):
    rclpy.init(args=args)

    client = JintClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info(response.output)
                break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
