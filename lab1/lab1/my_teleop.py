import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import curses
import sys

global msg

def getKey():
    global msg
    stdscr = curses.initscr()
    stdscr.addstr(0,0,msg)
    curses.cbreak()
    stdscr.keypad(1)
    curses.noecho()
    stdscr.timeout(100)
    key=stdscr.getch()
    if key != ord('q'):
    	return key
    else:
    	curses.endwin()
    	return key


class myTeleopNode(Node):
    def __init__(self):
        super().__init__("my_teleop")
        self.publisher_=self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_=self.create_timer(0.1, self.publish_news)
        self.get_logger().info("My Teleop has been started")
        self.declare_parameter("up",'w')
        self.declare_parameter("down",'s')
        self.declare_parameter("left",'a')
        self.declare_parameter("right",'d')
       
    def publish_news(self):
        twist = Twist()
        x = 0.0
        th = 0.0
        key=getKey()
        moveKeys = {
        	ord(self.get_parameter('up').get_parameter_value().string_value): (1,0),
        	ord(self.get_parameter('down').get_parameter_value().string_value): (-1,0),
        	ord(self.get_parameter('left').get_parameter_value().string_value): (0,1),
        	ord(self.get_parameter('right').get_parameter_value().string_value): (0,-1),
        		}
        if key in moveKeys.keys():
        	x = moveKeys[key][0]
        	th = moveKeys[key][1]
        else:
        	x = 0.0
        	th = 0.0
        	if key==ord('q'):
        		sys.exit(0)

        twist.linear.x = x*1.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th*2.0
        self.publisher_.publish(twist)
        

def main(args=None):
    global msg
    if args is None:
    	args = sys.argv
    rclpy.init(args=args)
    node = myTeleopNode()
    
    msg = "Poruszanie: \n"+node.get_parameter('up').get_parameter_value().string_value+" do przodu\n"
    msg = msg + node.get_parameter('down').get_parameter_value().string_value+" do tylu\n"
    msg = msg + node.get_parameter('left').get_parameter_value().string_value+" w lewo\n"
    msg = msg + node.get_parameter('right').get_parameter_value().string_value+" w prawo\n"
    print(msg)
    rclpy.spin(node)
    curses.endwin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
