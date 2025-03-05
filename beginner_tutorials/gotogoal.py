#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi


class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function to update the turtle's pose."""
        self.pose = data

    def euclidean_distance(self, goal_pose):
        """Calculate the Euclidean distance to the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def steering_angle(self, goal_pose):
        """Calculate the desired angle to the goal."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, target_angle, constant=4):
        """Calculate angular velocity."""
        angle_diff = target_angle - self.pose.theta

        # Normalize the angle difference to be between -pi and pi
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        return constant * angle_diff

    def rotate_to_goal(self, goal_pose):
        """Rotate to face the goal."""
        vel_msg = Twist()
        target_angle = self.steering_angle(goal_pose)

        while abs(target_angle - self.pose.theta) > 0.01:
            vel_msg.angular.z = self.angular_vel(target_angle)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            # Update target angle to prevent overshooting
            target_angle = self.steering_angle(goal_pose)

        # Stop rotation
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def linear_vel(self, goal_pose, constant=1.5):
        """Calculate linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def move_straight_to_goal(self, goal_pose, distance_tolerance):
        """Move straight to the goal."""
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) > distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = 0  # Ensure no angular movement
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop movement
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal(self):
        """Move the turtle to the specified goal."""
        goal_pose = Pose()

        # Get user input for goal position and tolerance
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))

        # Step 1: Rotate to face the goal
        self.rotate_to_goal(goal_pose)

        # Step 2: Move straight to the goal
        self.move_straight_to_goal(goal_pose, distance_tolerance)

        rospy.loginfo("Reached the goal!")

        # Keep the node alive until terminated
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
