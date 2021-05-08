import sys

from lab4_srv.srv import Oint
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
    	super().__init__('OINT_Client')
    	self.cli = self.create_client(Oint, 'oint_control_srv')
    	while not self.cli.wait_for_service(timeout_sec=2.0):
    		self.get_logger().info('service not available, waiting again...')
    	self.req = Oint.Request()
    	
    def send_request(self):
    
    	try:
    		self.req.x = int(sys.argv[1])
    		self.req.y = int(sys.argv[2])
    		self.req.z = int(sys.argv[3])
    		self.req.roll = int(sys.argv[4])
    		self.req.pitch = int(sys.argv[5])
    		self.req.yaw = int(sys.argv[6])
    		
    		self.req.time = int(sys.argv[7])
    		self.req.type = int(sys.argv[8])
    	except ValueError:
    		self.get_logger().info('ValueError')
    	self.future = self.cli.call_async(self.req)
	
def main(args=None):
    rclpy.init(args=args)

    client = OintClient()
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
