import rclpy
from rclpy.node import Node
import math
import time
from statistics import mean 

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class TurtleController(Node):
    def __init__(self):
        super().__init__("Controller_Node")
        self.clock = 0
        self.vel_pub = self.create_publisher(Twist, f'/cmd_vel', 10)
        self.goal_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback_listener,10)
        self.ranges = []
        #wall thresholds
        self.wall_dists = [1.0, 1.0, 1.0, 1.0]
        #flags for turning
        self.turnCW = False
        self.turnCCW = False
        self.finaldist = 0
        #flags for whether each corner is free
        self.back_rightfree=False
        self.front_rightfree=False
        self.back_leftfree=False
        self.front_leftfree = False


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

        #define different distances used
        self.ranges = list(msg.ranges)
        north_dist = mean(self.ranges[0:10] + self.ranges[350:360])
        north_min = min(self.ranges[0:20] + self.ranges[340:360])
        east_dist = mean(self.ranges[225:315])
        east_smaller = mean(self.ranges[260:270])
        east_min=min(self.ranges[225:315])
        west_dist = mean(self.ranges[45:135])
        west_smaller=mean(self.ranges[80:100])
        west_min = min(self.ranges[45:135])
        back_right=self.ranges[225:270]
        front_right=self.ranges[270:315]
        front_left=self.ranges[45:90]
        back_left = self.ranges[90:135]

        #check if each corner of the robot is free
        if any(value > 1 for value in back_right):
            self.back_rightfree=True
            #self.get_logger().info("BACK RIGHT FREE")
        else:
            self.back_rightfree=False
            
        if any(value > 1 for value in back_left):
            self.back_leftfree=True
            #self.get_logger().info("BACK LEFT FREE")
        else:
            self.back_leftfree=False
        if any(value > 1 for value in front_right):
            self.front_rightfree=True
            #self.get_logger().info("FRONT RIGHT FREE")
        else:
            self.front_rightfree=False
            
        if any(value > 1 for value in front_left):
            self.front_leftfree=True
            #self.get_logger().info("FRONT LEFT FREE")
        else:
            self.front_leftfree=False
        #positive if front right corner is closer to wall than back right corner
        right_diff=(mean(front_right) - mean(back_right))
        left_diff=(mean(front_left) - mean(back_left))

        self.get_logger().info("west_dist: %s" % str(west_smaller))
        self.get_logger().info("north_wall: %s" % str(north_min))
        self.get_logger().info("east_wall: %s" % str(east_min))
        
        if self.turnCCW==True:
            self.publish_vel(x=0.0,y=0.0,az=0.2)
            #check if front dist is within 0.08m of final dist, or if front dist >1.2 to exit turn
            if abs(mean(self.ranges[0:10]+self.ranges[350:360])-self.finaldist)<0.08 or mean(self.ranges[0:10]+self.ranges[350:360])>1.2:
                self.turnCCW=False
                self.get_logger().info("west_min: %s" % str(west_min))
                self.get_logger().info("STOP TURN")
        elif self.turnCW==True:
            self.publish_vel(x=0.0,y=0.0,az=-0.2)
            self.get_logger().info("dist diff %s" % str(abs(mean(self.ranges[0:10]+self.ranges[350:360])-self.finaldist)))
            #check if front dist is within 0.08m of final dist, or if front dist >1.2 to exit turn
            if abs(mean(self.ranges[0:10]+self.ranges[350:360])-self.finaldist)<0.08 or mean(self.ranges[0:10]+self.ranges[350:360])>1.2:
                self.turnCW=False
                self.get_logger().info("STOP TURN")
        elif north_dist < 0.51 and west_smaller >= east_dist: # something in front, left is more free turn left.
            self.get_logger().info("90 CCW")
            #set final front distance for CCW turn
            self.finaldist=west_smaller
            self.turnCCW=True 
        elif north_dist < 0.51 and east_dist > west_smaller: # something in front, right is more free, turn right.
            self.get_logger().info("90 CW")
            #set final front distance for CW turn
            self.finaldist=east_smaller
            self.turnCW=True   
        
        elif min(self.ranges) > 1.5: # exited the maze, ready to do 180 degree turn and stop.
            self.publish_vel(x=0.0,y=0.0,az=1.0)
            time.sleep(4.0)
            self.publish_vel(x=0.0,y=0.0,az=0.0)
            time.sleep(20)
        else:
            #follow right wall if right wall is closer
            if east_dist<west_dist:
                self.get_logger().info("Right diff: %s" % str(right_diff))
                #move away from wall if minimum right distance is too small
                if east_min<0.2:
                    self.publish_vel(x=0.08,y=0.0,az=0.1)
                    self.get_logger().info("something on right!")   
                #if the difference in the two angles on the right is greater than 0.025, adjust so robot is parallel with wall
                #if any right corner is free, do not adjust
                elif right_diff > 0.025 and self.front_rightfree==False and self.back_rightfree==False:
                    self.publish_vel(x=0.08,y=0.0,az=-0.3)
                    self.get_logger().info("correcting")
                    
                elif(right_diff < -0.025) and self.front_rightfree==False and self.back_rightfree==False:
                    self.publish_vel(x=0.08,y=0.0,az=0.3)
                    self.get_logger().info("correcting")
                else:
                    #if none of other cases are true, go foward
                    self.publish_vel(x=0.25,y=0.0,az=0.0)
            #follow left wall if left wall is closer
            else:
                #move away from wall if minimum left distance is too small
                if west_min<0.2:
                    self.publish_vel(x=0.08,y=0.0,az=-0.15)
                    self.get_logger().info("something on left!")
                #if the difference in the two angles on the left is greater than 0.025, adjust so robot is parallel with wall
                #if any left corner is free, do not adjust
                elif left_diff > 0.025 and self.front_leftfree==False and self.back_leftfree==False:
                    self.publish_vel(x=0.08,y=0.0,az=0.3)
                    self.get_logger().info("correcting")
                    
                elif(left_diff < -0.025) and self.front_leftfree==False and self.back_leftfree==False:
                    self.publish_vel(x=0.08,y=0.0,az=-0.3)
                    self.get_logger().info("correcting")
                else:
                    #if none of other cases are true, go foward
                    self.publish_vel(x=0.25,y=0.0,az=0.0)

def main(args=None):
    rclpy.init(args=args)
    
    node = TurtleController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()