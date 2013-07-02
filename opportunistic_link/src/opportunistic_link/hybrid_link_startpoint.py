#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.srv import *
import link_startpoint

class HybridLinkStartPoint:

    def __init__(self, input_topic_name, input_topic_type, aggregation_topic_name, transport_ctrl):
        rospy.loginfo("Starting HybridLinkStartPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.input_topic_name = input_topic_name
        self.aggregation_topic_name = aggregation_topic_name
        self.forward = False
        self.rate = float('infinity')
        self.last_msg = None
        self.looprate = rospy.Rate(10.0)
        self.rate_server = rospy.Service(rate_ctrl, RateControl, self.rate_cb)
        self.server = rospy.Service(transport_ctrl, LinkControl, self.link_cb)
        self.publisher = rospy.Publisher(self.aggregation_topic_name, SerializedMessage)
        self.subscriber = rospy.Subscriber(self.input_topic_name, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("...HybridLinkStartPoint loaded")
        while not rospy.is_shutdown():
            if (self.rate != float('infinity') and self.rate != 0.0 and self.last_msg != None and self.forward):
                self.publisher.publish(self.last_msg)
                self.last_msg = None
            self.looprate.sleep()

    def link_cb(self, request):
        if (request.Forward == True):
            self.forward = True
            rospy.loginfo("Set forwarding to TRUE")
        elif (request.Forward == False):
            self.forward = False
            rospy.loginfo("Set forwarding to FALSE")
        response = LinkControlResponse()
        response.State = self.forward
        return response

    def rate_cb(self, request):
        self.rate = abs(request.Rate)
        if (self.rate != 0.0 and self.rate != float('infinity')):
            self.looprate = rospy.Rate(self.rate)
        rospy.loginfo("Set rate to " + str(self.rate) + " - topic: " + self.input_topic_name)
        response = RateControlResponse()
        response.State = self.rate
        return response

    def sub_cb(self, msg):
        buff = StringIO.StringIO()
        msg.serialize(buff)
        serialized_msg = SerializedMessage()
        serialized_msg.TopicName = self.input_topic_name
        serialized_msg.TopicType = self.topic_package + "/" + self.topic_type
        serialized_msg.SerializedMessageData = buff.getvalue()
        buff.close()
        if (self.rate == float('infinity') and self.forward):
            self.publisher.publish(serialized_msg)
        else:
            self.last_msg = serialized_msg

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
    input_topic_name = rospy.get_param("~input_topic_name", "robot/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    transport_data = rospy.get_param("~transport_data", "opportunistic_link/link_data")
    transport_ctrl = rospy.get_param("~transport_ctrl", "opportunisitc_link/link_control")
    rate_ctrl = rospy.get_param("~rate_ctrl", "robot/test/rate")
    default_rate = rospy.get_param("~default_rate", float('infinity'))
    aggregation_topic = rospy.get_param("~aggregation_topic", "test/Aggregator")

    HybridLinkStartPoint(input_topic_name, topic_type, transport_data, transport_ctrl)
