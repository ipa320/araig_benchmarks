#!/usr/bin/env python

import rospy
import math
import threading
from araig_msgs.msg import BoolStamped
from std_msgs.msg import Float64 
from marvelmind_nav.msg import hedge_pos_a

mutex_stop_robot = threading.Lock()
mutex_robot_has_stopped = threading.Lock()

flag_stop_robot = False
flag_robot_has_stopped = False

msg_robot = hedge_pos_a()
msg_obstacle = hedge_pos_a()

def publisher_thread():
    global msg_robot
    global msg_obstacle
    ROBOT_ADDRESS = 1
    OBSTACLE_ADDRESS = 2
    pub_hedge = rospy.Publisher('/hedge_pos_a', hedge_pos_a, queue_size=10)
    rate = rospy.Rate(4)
    x = 0
    while not rospy.is_shutdown():
        sin = math.sin(math.radians(x))
        cos = math.cos(math.radians(x))

        msg_robot.address = ROBOT_ADDRESS
        msg_robot.timestamp_ms = rospy.Time.now().to_sec()
        msg_robot.x_m = sin
        pub_hedge.publish(msg_robot)

        msg_obstacle.address = OBSTACLE_ADDRESS
        msg_obstacle.timestamp_ms = rospy.Time.now().to_sec()
        msg_obstacle.x_m = cos
        pub_hedge.publish(msg_obstacle)

        x = 0 if x == 360 else x + 1
        rate.sleep()
    print("Killing thread")

def main():
    global msg_robot
    global msg_obstacle

    def cb_stop_robot(msg):
        global mutex_stop_robot
        global flag_stop_robot
        with mutex_stop_robot:
            flag_stop_robot = msg.data

    def cb_robot_has_stopped(msg):
        global mutex_robot_has_stopped
        global flag_robot_has_stopped
        with mutex_robot_has_stopped:
            flag_robot_has_stopped = msg.data

    rospy.init_node("mock_marvel",disable_signals=True)

    pub_result_robot = rospy.Publisher('/data/mock/braking_distance', Float64, queue_size=10)
    pub_result_obstacle = rospy.Publisher('/mock_sensor/result_obstacle', Float64, queue_size=10)
    rospy.Subscriber("/signal/runner/stop_robot", BoolStamped, cb_stop_robot)
    rospy.Subscriber("/signal/calc/robot_has_stopped", BoolStamped, cb_robot_has_stopped)

    skip_stop_robot = False
    skip_robot_has_stopped = False

    thread = threading.Thread(target=publisher_thread)
    thread.daemon = True
    thread.start()


    while not rospy.is_shutdown():
        stop_robot = False
        robot_has_stopped = False


        # Wait for first signal to go high
        while not rospy.is_shutdown() and not stop_robot:
            with mutex_stop_robot:
                stop_robot = flag_stop_robot

        # Save current position
        rospy.loginfo(rospy.get_name()+ " Stop robot raised")
        robot_displacement = msg_robot.x_m
        obstacle_displacement = msg_obstacle.x_m

        # Wait for second signal to go high
        while not rospy.is_shutdown() and not robot_has_stopped:
            with mutex_robot_has_stopped:
                robot_has_stopped = flag_robot_has_stopped

        if stop_robot and robot_has_stopped:
            rospy.loginfo(rospy.get_name()+ " Robot has stopped raised")
            robot_displacement = robot_displacement - msg_robot.x_m
            obstacle_displacement = obstacle_displacement - msg_obstacle.x_m
            pub_result_robot.publish(robot_displacement)
            pub_result_obstacle.publish(obstacle_displacement)

        # Wait for both signals to go low
        while not rospy.is_shutdown() and stop_robot:
            with mutex_stop_robot:
                stop_robot = flag_stop_robot
        while not rospy.is_shutdown() and robot_has_stopped:
            with mutex_robot_has_stopped:
                robot_has_stopped = flag_robot_has_stopped

            

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass