import rclpy #Import ROS 2 libary 
from rclpy.node import Node #Import Node class 
from geometry_msgs.msg import Twist #Import twist message
from turtlesim.msg import Pose #import pose message

class SimpleWallAvoidingRobot(Node):
    def __init__(self):
        super().__init__('simple_wall_avoiding_robot')

        #Publisher to send movement commands to our turtle
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        #Subscriber to recieve turtles positions
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)  

    #callback function triggered when a nose pose message is recieved
    def pose_callback(self, msg):  
        #Obtain turtle position
        robots_x_position = msg.x
        robots_y_position = msg.y

        #set boundaries for the wall 
        x_min = .5
        y_min = .5
        x_max = 10.5
        y_max = 10.5

        #distances from the wall or obstacles to trigger the turtle to rotate
        wall_threshold = .5
        obstacle_threshold = 1.0

        #Check for if turtle is near walls
        near_left_wall = robots_x_position < (x_min + wall_threshold)
        near_right_wall = robots_x_position > (x_max - wall_threshold)
        near_bottom_wall = robots_y_position < (y_min + wall_threshold)
        near_top_wall = robots_y_position > (y_max - wall_threshold)

        #Check if turtle is near obstacles 
        near_obstacle1 = abs(robots_x_position - 3.0) < obstacle_threshold and abs(robots_y_position - 3.0) < obstacle_threshold
        near_obstacle2 = abs(robots_x_position - 5.0) < obstacle_threshold and abs(robots_y_position - 5.0) < obstacle_threshold
        near_obstacle3 = abs(robots_x_position - 7.0) < obstacle_threshold and abs(robots_y_position - 3.0) < obstacle_threshold
        near_obstacle4 = abs(robots_x_position - 2.0) < obstacle_threshold and abs(robots_y_position - 7.0) < obstacle_threshold

        #Combine all the obstacles and walls into one list
        in_danger = (near_left_wall or near_right_wall or near_bottom_wall or near_top_wall or
                     near_obstacle1 or near_obstacle2 or near_obstacle3 or near_obstacle4)

        #Create a twist message to store velocity commands 
        twist_msg = Twist()

        #If the turtle runs into an obstacle or wall 
        if in_danger:
            twist_msg.linear.x = 2.0   # increased forward speed
            twist_msg.angular.z = 2.0  # increased turn speed
            self.get_logger().info('Rotating and moving forward to avoid obstacle or wall')

        #nothing is contacted and turtle is roaming freely    
        else:
            twist_msg.linear.x = 1.5   # increased forward speed
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving forward')

        self.publisher.publish(twist_msg)

def main():
    rclpy.init()
    node = SimpleWallAvoidingRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    node.publisher.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
