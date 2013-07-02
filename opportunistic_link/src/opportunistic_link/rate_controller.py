#!/usr/bin/python

#################################################
#                                               #
#   ROS Message Topic Rate Controller           #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.srv import *
import rate_controller

class RateController:

    def __init__(self, input_topic_name, topic_type, output_topic_name, rate_ctrl, default_rate):
        rospy.loginfo("Starting RateController...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(topic_type)
        self.dynamic_load(self.topic_package)
        self.input_topic_name = input_topic_name
        self.output_topic_name = output_topic_name
        self.rate = float('infinity')
        self.last_time = rospy.get_time()
        self.time_step = rospy.Duration(0.0)
        self.rate_server = rospy.Service(rate_ctrl, RateControl, self.rate_cb)
        self.publisher = rospy.Publisher(output_topic_name, eval(self.topic_type))
        self.subscriber = rospy.Subscriber(self.input_topic_name, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("...RateController loaded")
        while not rospy.is_shutdown():
            rospy.spin()

    def rate_cb(self, request):
        self.rate = request.Rate
        self.time_step = rospy.Duration(1.0 / self.rate)
        rospy.loginfo("Set rate to " + str(self.rate) + " - topic: " + self.topic_name)
        response = RateControlResponse()
        response.State = self.rate
        return response

    def sub_cb(self, msg):
        if (self.rate == float('infinity')):
            self.publisher.publish(msg)
        else:
            new_time = rospy.get_time()
            if (rospy.Duration(new_time - self.last_time) > self.time_step):
                self.last_time = new_time
                self.publisher.publish(msg)

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('rate_controller')
    input_topic_name = rospy.get_param("~input_topic_name", "robot/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    output_topic_name = rospy.get_param("~output_topic_name", "limited/robot/test")
    rate_ctrl = rospy.get_param("~rate_ctrl", "robot/test/rate")
    default_rate = rospy.get_param("~default_rate", float('infinity'))
    RateController(input_topic_name, topic_type, output_topic_name, rate_ctrl, default_rate)
