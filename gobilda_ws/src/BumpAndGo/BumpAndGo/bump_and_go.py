import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from enum import Enum
import math


class State(Enum):
    FOWARD = 1
    BACKWARD = 2
    TURN = 3
    STOP = 4



ROBOT_WIDTH = 0.5
ROBOT_LINEAR_SPEED = 1.0
ROBOT_ANGULAR_SPEED = 0.5
BUMP_DIST = 1.0







class BumpAndGo(Node):
    def __init__(self):
        super().__init__('bump_and_go')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            LaserScan,
            'scan',
            self.bumper_callback,
            10)
        
        self.velocity_msg = TwistStamped()
        self.laser_data = LaserScan()

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.state = State.FOWARD
        self.state_start_time = 0
        self.last_scan_time = self.get_clock().now()            

    def timer_callback(self):
        # this function will control publishing to the cmd_vel topic

        
        # call fsm
        vel = self.robot_controller()
        self.velocity_msg.twist.linear.x = vel[0]
        self.velocity_msg.twist.angular.z = vel[1]
        self.publisher_.publish(self.velocity_msg)
        
        
            
    def robot_controller(self) -> tuple[float, float]:
        robot_points = self.transform_lidar_to_robot()

        now = self.get_clock().now()
        # check for laser timeout
        if (self.state != State.STOP):
            elapsed_since_last_scan = (now - self.last_scan_time).nanoseconds / 1e9
            if (elapsed_since_last_scan > 2.0):
                self.get_logger().info('Laser scan timeout, stopping robot')
                self.state = State.STOP
                return (0.0, 0.0)
       

        if (self.state == State.FOWARD):
            self.get_logger().info('foward state')
            # see if there is object 1m ahead
            for point in robot_points:
                if (-ROBOT_WIDTH/2 <= point[1] <= ROBOT_WIDTH/2): 
                    if (0 <= point[0] <= BUMP_DIST):
                        # there is an object within 1m ahead of robot 
                        self.state = State.BACKWARD
                        self.state_start_time = self.get_clock().now()
                        return (0.0, 0.0)
            
            # robot is free to move foward
            return (ROBOT_LINEAR_SPEED, 0.0)

        elif (self.state == State.BACKWARD):
            self.get_logger().info('backward state')
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            if (elapsed >= 1.0):
                self.state = State.TURN
                self.get_logger().info('TURNING BC TIME')
                return (0.0, 0.0)
            for point in robot_points:
                if (-ROBOT_WIDTH/2 <= point[1] <= ROBOT_WIDTH/2): 
                    if (-BUMP_DIST <= point[0] <= 0):
                        # there is an object within 1m behind robot 
                        self.state = State.TURN
                        self.get_logger().info('TURNING BC OBJECT')
                        return (0.0, 0.0)
            # robot is free to move backwards 
            return (-ROBOT_LINEAR_SPEED, 0.0)


        elif (self.state == State.TURN):
            self.get_logger().info('turn state')
            # see if there is an opening of 4m infront of robot
            for point in robot_points:
                if (-ROBOT_WIDTH/2 <= point[1] <= ROBOT_WIDTH/2): 
                    if (0 <= point[0] <= 4):
                        # there is an object within 4m infront of robot 
                        return (0.0, ROBOT_ANGULAR_SPEED)
                
                # there is a free space for robot to move foward
            self.state = State.FOWARD
            return (0.0, 0.0)


        elif (self.state == State.STOP):
            self.get_logger().info('stop state')
            return (0.0, 0.0)


    def bumper_callback(self, msg:LaserScan):
        # this function will handle the laser scan data
        self.laser_data = msg
        self.last_scan_time = self.get_clock().now()
        


    # can make this better by only transforming points infront of robot
    # would need to know which angles corrspond to infront of robot


    # angle_min: -3.1241390705108643
    # angle_max: 3.1415927410125732
    # angle_increment: 0.005806980188935995


    def transform_lidar_to_robot(self):
        # this function will transform the lidar coordinates to robot coordinates
        x_lidar = 0.1
        y_lidar = -0.1
        theta_lidar = 0.0

        robot_points = []


        # need to select which i values actually matter
        for i,r in enumerate(self.laser_data.ranges):
            if math.isinf(r) or math.isnan(r):
                continue #invalid reading
            
            
            angle = self.laser_data.angle_min + i * self.laser_data.angle_increment
             
            # get point in robot frame 
            # negative 1 to account for robot center to edge of robot
            x_robot = x_lidar + r * math.cos(angle + theta_lidar)
            y_robot = y_lidar + r * math.sin(angle + theta_lidar)

            robot_points.append((x_robot, y_robot))
        return robot_points


       
        
        

def main(args=None):
    rclpy.init(args=args)
    node = BumpAndGo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
        

    

