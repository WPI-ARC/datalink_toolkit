#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from opportunistic_link.srv import *

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

class LinkEndPoint:

    def __init__(self, topic_name, output_topic_type, transport_namespace):
        rospy.loginfo("Starting LinkEndPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(output_topic_type)
        self.dynamic_load(self.topic_package)
        self.transport_namespace = transport_namespace
        self.topic_name = topic_name
        self.client = rospy.ServiceProxy(transport_namespace + "/link_control", LinkControl)
        self.subscriber = rospy.Subscriber(transport_namespace + "/link_data", eval(self.topic_type), self.sub_cb)
        self.subscriber_handler = SubscribeHandler(self.sub_connect, self.sub_disconnect)
        self.publisher = rospy.Publisher(self.topic_name, eval(self.topic_type), self.subscriber_handler)
        rospy.loginfo("...LinkEndPoint loaded")
        while not rospy.is_shutdown():
            rospy.spin()

    def sub_connect(self, num_subscribers):
        if (num_subscribers > 1):
            rospy.loginfo("Additional subscriber connected, no need to change link state")
        elif (num_subscribers > 0):
            link_change = LinkControlRequest()
            link_change.Forward.data = True
            rospy.loginfo("First subscriber connected, starting link data flow...")
            self.client.call(link_change)
            rospy.loginfo("...link data flow started")

    def sub_disconnect(self, num_subscribers):
        if (num_subscribers < 1):
            link_change = LinkControlRequest()
            link_change.Forward.data = False
            rospy.loginfo("Last subscriber disconnected, stopping link data flow...")
            self.client.call(link_change)
            rospy.loginfo("...link data flow stopped")

    def sub_cb(self, msg):
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
    rospy.init_node('link_endpoint')
    topic_name = rospy.get_param("~topic_name", "workstation/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    transport_namespace = rospy.get_param("~transport_namespace", "opportunistic_link")
    LinkEndPoint(topic_name, topic_type, transport_namespace)
