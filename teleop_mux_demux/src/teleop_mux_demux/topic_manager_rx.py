#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   ROS Message demultiplexer & deserializer        #
#                                                   #
#####################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.msg import *
from teleop_msgs.srv import *

import topic_manager_rx

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

class TopicHandlerRX:

    def __init__(self, original_topic_name, self.real_topic_type, republish_namespace):
        self.republish_namespace = republish_namespace
        self.topic_name = topic_name
        self.client = rospy.ServiceProxy(self.topic_name + "/link_control", LinkControl)
        link_change = LinkControlRequest()
        link_change.Forward = False
        rospy.loginfo("Setting transport link state...")
        self.client.call(link_change)
        rospy.loginfo("...link state set - topic: " + self.topic_name)
        self.subscriber_handler = SubscribeHandler(self.sub_connect, self.sub_disconnect)
        real_publisher_name = (self.republish_namespace + "/" + self.original_topic_name).lstrip("/")
        self.publisher = rospy.Publisher(self.topic_name, eval(self.topic_type), self.subscriber_handler)
        rospy.loginfo("Started topic handler - topic: " + self.topic_name)

    def sub_connect(self, num_subscribers):
        if (num_subscribers > 1):
            rospy.loginfo("Additional subscriber connected, no need to change link state - topic: " + self.topic_name)
        elif (num_subscribers > 0):
            link_change = LinkControlRequest()
            link_change.Forward = True
            rospy.loginfo("First subscriber connected, starting link data flow...")
            self.client.call(link_change)
            rospy.loginfo("...link data flow started - topic: " + self.topic_name)

    def sub_disconnect(self, num_subscribers):
        if (num_subscribers < 1):
            link_change = LinkControlRequest()
            link_change.Forward = False
            rospy.loginfo("Last subscriber disconnected, stopping link data flow...")
            self.client.call(link_change)
            rospy.loginfo("...link data flow stopped - topic: " + self.topic_name)

    def republish(self, serialized):
        new_msg = self.real_topic_type()
        new_msg.deserialize(serialized)
        self.publisher.publish(msg)

class TopicManagerRX:

    def __init__(self, aggregated_topic_in, namespace):
        self.available_topics = {}
        self.topic_handlers = {}
        self.namespace = namespace
        self.imported_packages = ["std_msgs", "teleop_msgs"]
        self.subscriber = rospy.Subscriber(aggregated_topic_in, AggregatedMessages, self.sub_cb)
        rospy.loginfo("Loaded topic manager [RX]")
        while not rospy.is_shutdown():
            rospy.spin()

    def sub_cb(self, msg):
        for message in msg.Messages:
            [topic_type, topic_package] = self.extract_type_and_package(message.TopicType)
            # First, check to see if the message is a system update message and reconfigure accordingly
            if (message.TopicType == "teleop_msgs/TopicUpdate"):
                # Deserialize the message
                update = TopicUpdate()
                update.deserialize(message.SerializedMessageData)
                # Make sure every topic has an endpoint and add one if it doesn't
                for topic_name in update.TopicNames:
                    try:
                        existing_type = self.available_topics[topic_name]
                        if (existing_type != topic_info[1]):
                            rospy.logerr("Already watching topic but the type has changed!")
                            self.available_topics[topic_name] = topic_info[1]
                            self.topic_handlers[topic_name] = self.build_new_handler(topic_name, topic_info[1])
                    except:
                        rospy.loginfo("Adding handler to a newly available topic: " + topic_name)
                        self.available_topics[topic_name] = topic_info[1]
                        self.topic_handlers[topic_name] = self.build_new_handler(topic_name, topic_info[1])
                rospy.loginfo("Handlers update from system update message")
            else:
                try:
                    self.topic_handlers[message.TopicName].republish(message.SerializedMessageData)
                except:
                    rospy.logerr("Received message data from a topic that isn't being handled - topic: " + message.TopicName + " - type: " + message.TopicType)

    def build_new_handler(self, topic_name, full_topic_type):
        [topic_type, topic_package] = self.extract_type_and_package(full_topic_type)
        if (topic_package not in self.imported_packages):
            self.dynamic_load(topic_package)
            self.imported_packages.append(topic_package)
        return TopicHandlerRX(topic_name, eval(topic_type), self.namespace)

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('topic_manager_rx')
    aggregated_topic = rospy.get_param("~aggregated_topic", "Aggregated")
    TopicManagerRX(aggregated_topic, rospy.get_namespace())
