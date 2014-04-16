#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from datalink_msgs.srv import *
import link_startpoint

class SubscribeHandler:

    def __init__(self, connect_cb, disconnect_cb):
        self.connect_cb = connect_cb
        self.disconnect_cb = disconnect_cb
        self.num_subscribers = 0

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self.num_subscribers += 1
        self.connect_cb(self.num_subscribers)

    def peer_unsubscribe(self, topic_name, num_peers):
        self.num_subscribers += -1
        self.disconnect_cb(self.num_subscribers)

class LinkStartPoint:

    def __init__(self, input_topic_name, input_topic_type, transport_data, transport_ctrl):
        rospy.loginfo("Starting LinkStartPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.input_topic_name = input_topic_name
        self.forward = False
        self.server = rospy.Service(transport_ctrl, LinkControl, self.link_cb)
        self.subscriber_handler = SubscribeHandler(self.sub_connect, self.sub_disconnect)
        self.publisher = rospy.Publisher(transport_data, eval(self.topic_type), self.subscriber_handler, latch=True)
        self.subscriber = rospy.Subscriber(self.input_topic_name, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("...LinkStartPoint loaded")
        while not rospy.is_shutdown():
            rospy.spin()

    def sub_connect(self, num_subscribers):
        if (num_subscribers > 1):
            rospy.logerr("Multiple subscribers to the transport datalink. This is not safe behavior!")
        elif (num_subscribers > 0):
            rospy.loginfo("Link endpoint connected")

    def sub_disconnect(self, num_subscribers):
        if (num_subscribers < 1):
            self.forward = False
            rospy.loginfo("Link endpoint disconnected")

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

    def sub_cb(self, msg):
        if (self.forward):
            rospy.logdebug("Forwarding message data")
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
    input_topic_name = rospy.get_param("~input_topic_name", "test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    transport_data = rospy.get_param("~transport_data", "test/transport")
    transport_ctrl = rospy.get_param("~transport_ctrl", "test/ctrl")
    LinkStartPoint(input_topic_name, topic_type, transport_data, transport_ctrl)
