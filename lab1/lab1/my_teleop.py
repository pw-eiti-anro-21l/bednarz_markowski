import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)
msg = """
Poruszanie:
w-do przodu 
a- w lewo
s- do dolu
d- w prawo
CTRL-C wyjscie
"""

moveKeys = {
		'w':(1,0),
		'a':(0,1),
		'd':(0,-1),
		's':(-1,0),
		'W':(1,0),
		'A':(0,1),
		'D':(0,-1),
		'S':(-1,0),
	       }
def getKey():
    tty.setraw(sys.stdin.fileno())
    #select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class myTeleopNode(Node):
    def __init__(self):
        super().__init__("my_teleop")
        self.publisher_=self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_=self.create_timer(0.1, self.publish_news)
        self.get_logger().info("My Teleop has been started")

    def publish_news(self,key):
        twist = Twist()
        x = 0.0
        th = 0.0
        #key=getKey()
        if key in moveKeys.keys():
        	x = moveKeys[key][0]
        	th = moveKeys[key][1]
        else:
        	x = 0.0
        	th = 0.0

        twist.linear.x = x*0.5
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th*1.0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = myTeleopNode()
    print(msg)
    #rclpy.spin(node)
    i=0
    while i==0:
    	key=getKey()
    	if not key=='\x03':
    		node.publish_news(key)
    	else:
    		i=1
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
