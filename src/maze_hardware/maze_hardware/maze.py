import rclpy
from rclpy.node import Node
import math
import time
import signal
import functools

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class TurtleController(Node):
    def __init__(self):
        super().__init__("Controller_Node")
        self.clock = 0
        self.vel_pub = self.create_publisher(Twist, f'/cmd_vel', 10)
        self.goal_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback_listener_new,10)
        self.initial = False
        self.ranges = []
        self.allign = False
        self.wall_dists = [0.3, 0.3, 0.3, 0.3]
        self.searching = False

    def end_signal_handler(self, sig, frame):
        self.publish_vel(x=0.0, y=0.0, az=0.0)
        rclpy.shutdown()

    def get_adjusted_min_in_range(self, arr, min, max):
        mid = (min + max) / 2
        min_val = 1000
        for i in range(max - min):
            curr_val = arr[i + min]
            curr_val = curr_val * math.cos((mid - (i + min)) * 2.0 * math.pi / 720)
            if curr_val < min_val:
                min_val = curr_val
        return min_val

    def publish_vel(self, x: float = 0.0, y: float = 0.0, az: float = 0.0):
        vel_msg = Twist()
        lin_msg = Vector3()
        ang_msg = Vector3()
        
        lin_msg.x = x
        lin_msg.y = y 
        lin_msg.z = 0.0
        ang_msg.x = 0.0
        ang_msg.y = 0.0
        ang_msg.z = az
        
        vel_msg.linear = lin_msg
        vel_msg.angular = ang_msg
        self.vel_pub.publish(vel_msg)

    def scan_callback_listener(self, msg: LaserScan):
        self.ranges = list(msg.ranges)
        
        right_dist = min(self.ranges[110:250])
        front_dist = min(self.ranges[320:400])
        left_dist = min(self.ranges[450:630])

        self.get_logger().info("left_dist: %s" % str(left_dist))
        self.get_logger().info("front_dist: %s" % str(front_dist))
        self.get_logger().info("right_dist: %s" % str(right_dist))

        if self.allign ==True:
            self.get_logger().info("performing algorithm")
            a=self.ranges[203]
            b=self.ranges[111]

            diff=a-b
            self.get_logger().info("diff is %s" % str(diff))
            val=0.05
            if diff>val:
                self.get_logger().info("in algo, turning right")
                self.publish_vel(x=0.0,y=0.0,az=-0.1)
            elif diff<-val:
                self.get_logger().info("in algo, turning left")
                self.publish_vel(x=0.0,y=0.0,az=0.1)
            else:
                self.get_logger().info("equal going straight")
                self.publish_vel(x=0.1,y=0.0,az=0.0)
                self.allign = False
                return 1
        else:
            if left_dist < 0.2 and front_dist < 0.5:
                self.get_logger().info("caution! bumping on left!!!!")
                self.publish_vel(x=0.0,y=0.0,az=-0.25)
                time.sleep(0.5)
                self.allign = True
            elif right_dist < 0.2 and front_dist < 0.5:
                self.get_logger().info("caution! bumping on right!!!!")
                self.publish_vel(x=0.0,y=0.0,az=0.25)
                time.sleep(0.5)
                self.allign = True
            if front_dist > 0.5:
                if right_dist < 0.5:
                    self.get_logger().info("nothing in front, continuing")
                    self.publish_vel(x=0.25, y=0.0, az=0.0)
                else:
                    self.publish_vel(x=0.0, y=0.0, az=-0.15)
                    time.sleep(0.5)
                    self.allign = True
            
            else:
                # elif right_dist > left_dist:
                #     self.get_logger().info("turning right")
                #     self.publish_vel(x=0.0,y=0.0,az=-0.25)
                # if left_dist == right_dist:
                #     self.get_logger().info("rotating out")
                #     self.publish_vel(x=0.0,y=0.0,az=0.75)
                self.get_logger().info("turning left")
                self.publish_vel(x=0.0,y=0.0,az=0.15)
                time.sleep(0.5)
                self.allign = True


    def scan_callback_listener_new(self, msg: LaserScan):

        self.ranges = list(msg.ranges)
        
        north_dist = min(self.ranges[270:450])
        east_dist = min(self.ranges[90:270])
        south_dist = min(self.ranges[630:720] + self.ranges[0:90])
        west_dist = min(self.ranges[450:630])
        # north_dist = self.get_adjusted_min_in_range(self.ranges, 270, 450)
        # east_dist = self.get_adjusted_min_in_range(self.ranges, 90, 270)
        # south_dist = min(self.get_adjusted_min_in_range(self.ranges, 630, 720), self.get_adjusted_min_in_range(self.ranges, 0, 90))
        # west_dist = self.get_adjusted_min_in_range(self.ranges, 450, 630)
        self.get_logger().info("north_dist: %s" % str(north_dist))
        self.get_logger().info("east_dist: %s" % str(east_dist))
        self.get_logger().info("south_dist: %s" % str(south_dist))
        self.get_logger().info("west_dist: %s" % str(west_dist))

        north_wall = north_dist < self.wall_dists[0]
        east_wall = east_dist < self.wall_dists[1]
        south_wall = south_dist < self.wall_dists[2]
        west_wall = west_dist < self.wall_dists[3]
        self.get_logger().info("north_wall: %s" % str(north_wall))
        self.get_logger().info("east_wall: %s" % str(east_wall))
        self.get_logger().info("south_wall: %s" % str(south_wall))
        self.get_logger().info("west_wall: %s" % str(west_wall))
        
        if ((not east_wall) and south_wall):
            # CW 90
            self.get_logger().info("CW 90")
            self.publish_vel(x=0.0,y=0.0,az=-0.5)
        elif ((north_wall and (not south_wall) and west_wall) or ((not east_wall) and (not south_wall) and west_wall)):
            # CW 180
            self.get_logger().info("CW 180")
            self.publish_vel(x=0.0,y=0.0,az=-0.8)
        elif ((north_wall and (not south_wall) and (not west_wall)) or (north_wall and east_wall and south_wall)):
            # CCW 90
            self.get_logger().info("CCW 90")
            self.publish_vel(x=0.0,y=0.0,az=0.8)
        elif ((not north_wall) and (not south_wall) and (not west_wall)):
            # Follow
            self.get_logger().info("Follow")
            if(abs(min(self.ranges[180:270]) - min(self.ranges[90:180])) < 0.06):
                self.publish_vel(x=0.08,y=0.0,az=-0.3)
            else:
                self.publish_vel(x=0.08,y=0.0,az=0.8)
        else:
            # F
            self.get_logger().info("F")
            self.publish_vel(x=0.15,y=0.0,az=0.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = TurtleController()

    signal.signal(signal.SIGINT, node.end_signal_handler)
    signal.signal(signal.SIGTERM, node.end_signal_handler)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()