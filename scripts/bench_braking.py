#!/usr/bin/env python

import threading
import numpy
import rospy
from araig_msgs.msg import BoolStamped
from std_msgs.msg import Float64
# This is the base test runner from araig_test_runners:
from base.base_runner import TestBase 

""""
BASE INTERFACES
self._input_interface = {
    "robot_has_stopped"    : "/signal/calc/robot_has_stopped",
    "start_test"           : "/signal/ui/start_test",
    "interrupt_test"       : "/signal/ui/interrupt_test"
}

self._output_interface = {
    "start_robot"     : "/signal/runner/start_robot",
    "test_completed"  : "/signal/runner/test_completed",
    "test_failed"     : "/signal/runner/test_failed",
    "test_succeeded"  : "/signal/runner/test_succeeded"
}
"""

class BenchBraking(TestBase):

    def __init__(self, rate):
        extend_subscribers_dict = {
        }
        extend_publishers_dict = {
            "robot_has_stopped"    : "/signal/calc/robot_has_stopped",
            "stop_robot"     : "/signal/runner/stop_robot"
        }

        self._lock_calc_braking_distance = threading.Lock()
        self._calc_braking_distance = 0.0
        self._lock_sensor_braking_distance = threading.Lock()
        self._sensor_braking_distance = 0.0
        self._number_of_loops = 20

        rospy.Subscriber("/data/calc/braking_distance", Float64, self.cb_calc_braking_distance)
        rospy.Subscriber("/data/mock/braking_distance", Float64, self.cb_sensor_braking_distance)
        
        super(BenchBraking, self).__init__(extend_subscribers_dict, extend_publishers_dict, rate = rate)
        rospy.sleep(rospy.Duration(1.5)) # Wait for other nodes to spawn
        self.main()

    def sleep_safe(self, duration):
        start = rospy.Time.now()
        while (rospy.Time.now() - start <= duration) and not rospy.is_shutdown():
            pass

    def cb_calc_braking_distance(self, msg):
        with self._lock_calc_braking_distance:
            self._calc_braking_distance = msg.data

    def cb_sensor_braking_distance(self, msg):
        with self._lock_sensor_braking_distance:
            self._sensor_braking_distance = msg.data


    def main(self):
        avg_error = 0.0
        avg_accuracy = 0.0
        error_list = []
        accuracy_list = []
        for itr in range(0,self._number_of_loops):
            # Emulate runner sends stop.
            rospy.loginfo(rospy.get_name() + ": -----------------------Trial number {} ---------------- ".format(itr))
            self._publishers["stop_robot"].publish(self.buildNewBoolStamped(True))

            # Emulate robot braking to stop
            rospy.sleep(rospy.Duration(0.5))

            # Emulate robot has stopped
            self._publishers["robot_has_stopped"].publish(self.buildNewBoolStamped(True))
            rospy.sleep(rospy.Duration(0.5))
            self._publishers["stop_robot"].publish(self.buildNewBoolStamped(False))
            self._publishers["robot_has_stopped"].publish(self.buildNewBoolStamped(False))

            # Wait for results
            rospy.sleep(rospy.Duration(0.5))

            # Check the resulting difference from mock sensor and calculator
            with self._lock_calc_braking_distance:
                calc_braking_distance = self._calc_braking_distance
            with self._lock_sensor_braking_distance:
                sensor_braking_distance = self._sensor_braking_distance

            rospy.loginfo(rospy.get_name()+ ": Calc dist: {} | Sens dist: {}".format(calc_braking_distance, sensor_braking_distance))

            error = abs( abs(calc_braking_distance) - abs(sensor_braking_distance))
            accuracy = 100 - abs(error / sensor_braking_distance)*100
            avg_error = ( avg_error + error) / 2
            avg_accuracy = ( avg_accuracy + accuracy )/2
            error_list.append(error)
            accuracy_list.append(accuracy)

            rospy.loginfo(rospy.get_name()+ ": Error: {} Accuracy: {}".format(error, accuracy))
            rospy.loginfo(rospy.get_name()+ ": Rolling Avg Err: {} Rolling Avg Acc: {}".format(avg_error, avg_accuracy))
            print("")

        rospy.loginfo(rospy.get_name()+ ": Error STD: {} Accuracy STD: {}"
            .format( numpy.std(error_list) ,numpy.std(accuracy_list) ))
        rospy.loginfo(rospy.get_name()+ ": Error AVG: {} Accuracy AVG: {}"
            .format( numpy.average(error_list) ,numpy.average(accuracy_list) ))
        rospy.loginfo(rospy.get_name()+ ": Error Max Dev: {} Accuracy Max Dev: {}"
            .format( max(abs(el - numpy.average(error_list)) for el in error_list),
                     max(abs(el - numpy.average(accuracy_list)) for el in accuracy_list) ))
                
if __name__ == "__main__":
    try:
        rospy.init_node("test1_runner", disable_signals=True)
        test = BenchBraking(100)
    except rospy.ROSException:
        pass