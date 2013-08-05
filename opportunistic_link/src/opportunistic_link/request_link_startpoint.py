#!/usr/bin/python

#################################################
#                                               #
#   Combined rate and link controller           #
#                                               #
#################################################

import rospy
import threading
import StringIO
from std_msgs.msg import *
from teleop_msgs.msg import *
from teleop_msgs.srv import *
import request_link_startpoint

class RequestLinkStartPoint:

    def __init__(self, input_topic_name, input_topic_type, request_service):
        rospy.loginfo("Starting RequestLinkStartPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.input_topic_name = input_topic_name
        self.last_msg = None
        self.request_server = rospy.Service(request_service, RequestMessage, self.request_cb)
        self.subscriber = rospy.Subscriber(self.input_topic_name, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("...RequestLinkStartPoint loaded")
        rospy.spin()

    def request_cb(self, request):
        response = RequestMessageResponse()
        if (self.last_msg != None):
            response.available = True
            response.message_data = self.last_msg
            self.last_msg = None
        else:
            response.available = False
            response.message_data = ''
        return response

    def sub_cb(self, msg):
        buff = StringIO.StringIO()
        msg.serialize(buff)
        self.last_msg = buff.getvalue()
        buff.close()

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('request_link_startpoint')
    input_topic_name = rospy.get_param("~input_topic_name", "test")
    input_topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    request_service = rospy.get_param("~request_service", "test/req")
    RequestLinkStartPoint(input_topic_name, input_topic_type, request_service)
