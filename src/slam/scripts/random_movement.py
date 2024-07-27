#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import random
import sys
import time


class RandomMover:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.robot_name}/cmd_vel', Twist, queue_size=10)
        rospy.init_node(f'random_mover_{self.robot_name}', anonymous=True)
        self.current_twist = Twist()
        self.target_twist = Twist()
        self.change_frequency = 5  # Change target velocity every 5 seconds

    def move_randomly(self):
        rate = rospy.Rate(10)  # 10 Hz
        last_change_time = time.time()

        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - last_change_time > self.change_frequency:
                self.set_new_target_velocity()
                last_change_time = current_time

            self.update_velocity()
            self.cmd_vel_pub.publish(self.current_twist)
            rate.sleep()

    def set_new_target_velocity(self):
        # Linear velocity
        self.target_twist.linear.x = random.uniform(-0.5, 0.5)
        # Angular velocity
        self.target_twist.angular.z = random.uniform(-1.0, 1.0)

    def update_velocity(self):
        # Gradually change the velocity to the target velocity
        self.current_twist.linear.x += (self.target_twist.linear.x -
                                        self.current_twist.linear.x) * 0.1
        self.current_twist.angular.z += (self.target_twist.angular.z -
                                         self.current_twist.angular.z) * 0.1


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: random_movement.py <robot_name>")
        sys.exit(1)

    robot_name = sys.argv[1]
    mover = RandomMover(robot_name)
    rospy.loginfo(f'Starting random movement for {robot_name}')
    mover.move_randomly()
