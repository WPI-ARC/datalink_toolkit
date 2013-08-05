#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.srv import *
import request_link_endpoint

class SubscribeHandler:

    def __init__(self, connect_cb, disconnect_cb):
        self.connect_cb = connect_cb
        self.disconnect_cb = disconnect_cb
        self.num_subscribers = 0

    def peer_subscribe(self, output_topic_name, topic_publish, peer_publish):
        self.num_subscribers += 1
        self.connect_cb(self.num_subscribers)

    def peer_unsubscribe(self, output_topic_name, num_peers):
        self.num_subscribers += -1
        self.disconnect_cb(self.num_subscribers)

class RequestLinkEndPoint:

    def __init__(self, output_topic_name, topic_type, request_service, default_rate, rebuild_delay, latched):
        rospy.loginfo("Starting RequestLinkEndPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(topic_type)
        self.dynamic_load(self.topic_package)
        self.real_topic_type = eval(self.topic_type)
        self.default_rate = rospy.Rate(default_rate)
        self.latched = latched
        self.active = False
        self.output_topic_name = output_topic_name
        self.request_service = request_service
        self.client = rospy.ServiceProxy(self.request_service, RequestMessage)
        self.subscriber_handler = SubscribeHandler(self.sub_connect, self.sub_disconnect)
        self.publisher = rospy.Publisher(self.output_topic_name, eval(self.topic_type), self.subscriber_handler, latch=self.latched)
        rospy.loginfo("...RequestLinkEndPoint loaded")
        while not rospy.is_shutdown():
            if (self.active):
                try:
                    response = self.client.call()
                    if (response.available):
                        new_msg = self.real_topic_type()
                        new_msg.deserialize(response.message_data)
                        self.publisher.publish(new_msg)
                    else:
                        rospy.logwarn("Requested message, but none available")
                except:
                    rospy.logerr("Request failed, trying to rebuild connection...")
                    rospy.sleep(rebuild_delay)
                    self.client = rospy.ServiceProxy(self.request_service, RequestMessage)
            self.default_rate.sleep()

    def sub_connect(self, num_subscribers):
        if (num_subscribers > 1):
            rospy.loginfo("Additional subscriber connected, no need to change link state")
        elif (num_subscribers > 0):
            self.active = True
            rospy.loginfo("New subscriber connected, setting to ACTIVE")

    def sub_disconnect(self, num_subscribers):
        if (num_subscribers < 1):
            self.active = False
            rospy.loginfo("Last subscriber disconnected, setting to INACTIVE")

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('request_link_endpoint')
    output_topic_name = rospy.get_param("~output_topic_name", "relay/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    request_service = rospy.get_param("~request_service", "test/req")
    default_rate = rospy.get_param("~default_rate", 20.0)
    rebuild_delay = rospy.get_param("~rebuild_delay", 5.0)
    latched = rospy.get_param("~latched", False)
    RequestLinkEndPoint(output_topic_name, topic_type, request_service, default_rate, rebuild_delay, latched)