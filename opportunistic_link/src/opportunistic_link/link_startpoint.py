#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from opportunistic_link.srv import *

class LinkStartPoint:

    def __init__(self, topic_name, input_topic_type, transport_namespace):
        rospy.loginfo("Starting LinkStartPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.transport_namespace = transport_namespace
        self.topic_name = topic_name
        self.forward = False
        self.server = rospy.Service(transport_namespace + "/link_control", LinkControl, self.link_cb)
        self.publisher = rospy.Publisher(transport_namespace + "/link_data", eval(self.topic_type))
        self.subscriber = rospy.Subscriber(self.topic_name, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("...LinkStartPoint loaded")
        while not rospy.is_shutdown():
            rospy.spin()

    def link_cb(self, request):
        if (request.Forward == True):
            self.forward = True
            rospy.loginfo("Set forwarding to TRUE")
        else:
            self.forward = False
            rospy.loginfo("Set forwarding to FALSE")
        response = LinkControlResponse()
        response.State = self.forward
        return response

    def sub_cb(self, msg):
        if (self.forward):
            rospy.loginfo("Forwarding message data")
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
    rospy.init_node('link_startpoint')
    topic_name = rospy.get_param("~topic_name", "test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    transport_namespace = rospy.get_param("~transport_namespace", "opportunistic_link")
    LinkStartPoint(topic_name, topic_type, transport_namespace)
