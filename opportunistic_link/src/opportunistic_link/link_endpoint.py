#!/usr/bin/python

#################################################
#                                               #
#   Opportunistic message forwarder             #
#                                               #
#################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.srv import *
import link_endpoint

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

class LinkEndPoint:

    def __init__(self, output_topic_name, output_topic_type, transport_data, transport_ctrl, latched, timeout=10.0):
        rospy.loginfo("Starting LinkEndPoint...")
        [self.topic_type, self.topic_package] = self.extract_type_and_package(output_topic_type)
        self.dynamic_load(self.topic_package)
        self.timeout = timeout
        self.latched = latched
        self.link_forward = False
        self.output_topic_name = output_topic_name
        self.transport_ctrl = transport_ctrl
        self.transport_data = transport_data
        self.client = rospy.ServiceProxy(self.transport_ctrl, LinkControl)
        link_change = LinkControlRequest()
        link_change.Forward = self.link_forward
        rospy.loginfo("Setting transport link state...")
        self.call_client(link_change)
        rospy.loginfo("...link state set")
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.connection_check, oneshot=True)
        self.subscriber = rospy.Subscriber(self.transport_data , eval(self.topic_type), self.sub_cb)
        self.subscriber_handler = SubscribeHandler(self.sub_connect, self.sub_disconnect)
        self.publisher = rospy.Publisher(self.output_topic_name, eval(self.topic_type), self.subscriber_handler, latch=self.latched)
        rospy.loginfo("...LinkEndPoint loaded")
        while not rospy.is_shutdown():
            rospy.spin()

    def connection_check(self, event):
        rospy.loginfo("Rebuilding service and subscriber...")
        try:
            self.client.close()
            self.subscriber.unregister()
        except:
            rospy.logwarn("Error trying to close service and subscriber")
        rospy.sleep(1.0)
        try:
            self.client = rospy.ServiceProxy(self.transport_ctrl, LinkControl)
            self.subscriber = rospy.Subscriber(self.transport_data , eval(self.topic_type), self.sub_cb)
            link_change = LinkControlRequest()
            link_change.Forward = self.link_forward
            self.call_client(link_change)
            rospy.loginfo("Reconnected link")
        except:
            rospy.logerr("Link appears to be down!")
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.connection_check, oneshot=True)

    def sub_connect(self, num_subscribers):
        if (num_subscribers > 1):
            rospy.loginfo("Additional subscriber connected, no need to change link state")
        elif (num_subscribers > 0):
            self.link_forward = True
            link_change = LinkControlRequest()
            link_change.Forward = self.link_forward
            rospy.loginfo("First subscriber connected, starting link data flow...")
            self.call_client(link_change)
            rospy.loginfo("...link data flow started")

    def sub_disconnect(self, num_subscribers):
        if (num_subscribers < 1):
            self.link_forward = False
            link_change = LinkControlRequest()
            link_change.Forward = self.link_forward
            rospy.loginfo("Last subscriber disconnected, stopping link data flow...")
            self.call_client(link_change)
            rospy.loginfo("...link data flow stopped")

    def call_client(self, request):
        try:
            response = self.client.call(request)
            return response
        except:
            rospy.logerr("Link control service call failed to connect")
            return None

    def sub_cb(self, msg):
        self.timer.shutdown()
        self.publisher.publish(msg)
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.connection_check, oneshot=True)

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
    output_topic_name = rospy.get_param("~output_topic_name", "relay/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    transport_data = rospy.get_param("~transport_data", "test/transport")
    transport_ctrl = rospy.get_param("~transport_ctrl", "test/ctrl")
    latched = rospy.get_param("~latched", False)
    LinkEndPoint(output_topic_name, topic_type, transport_data, transport_ctrl, latched)
