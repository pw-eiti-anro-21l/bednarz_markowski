import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('steer_fwd', 'w')
        self.declare_parameter('steer_bcd', 's')
        self.declare_parameter('steer_right', 'd')
        self.declare_parameter('steer_left', 'a')

    def timer_callback(self):
        my_param = self.get_parameter('steer_fwd').get_parameter_value().string_value
        my_param += self.get_parameter('steer_bcd').get_parameter_value().string_value
        my_param += self.get_parameter('steer_right').get_parameter_value().string_value
        my_param += self.get_parameter('steer_left').get_parameter_value().string_value
        self.get_logger().info('steering: %s' % my_param)

        #my_new_param = rclpy.parameter.Parameter(
        #    'steer_fwd',
        #    rclpy.Parameter.Type.STRING,
        #    'w'
        #)
        #all_new_parameters = [my_new_param]
        #self.set_parameters(all_new_parameters)

def main(args=None):
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
