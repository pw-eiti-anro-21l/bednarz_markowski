import sys

from lab4_srv.srv import Figure
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
    	super().__init__('OINT_Figure_Client')
    	self.cli = self.create_client(Figure, 'oint_figure_srv')
    	while not self.cli.wait_for_service(timeout_sec=2.0):
    		self.get_logger().info('service not available, waiting again...')
    	self.req = Figure.Request()
    	
    def send_request(self):
    
    	try:
    		self.req.param_a = float(sys.argv[2])
    		self.req.param_b = float(sys.argv[3])
    		
    		self.req.time = float(sys.argv[4])
    		self.req.figure = sys.argv[1]
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
