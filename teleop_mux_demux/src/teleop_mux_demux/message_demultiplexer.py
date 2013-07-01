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
import message_demultiplexer

class MessageDemultiplexer:

    def __init__(self, aggregated_topic_in, namespace):
        self.topic_pubs = {}
        self.namespace = namespace
        self.imported_packages = ["std_msgs", "teleop_msgs"]
        self.subscriber = rospy.Subscriber(aggregated_topic_in, AggregatedMessages, self.sub_cb)
        rospy.loginfo("Loaded Demultiplexer+Deserializer")
        while not rospy.is_shutdown():
            rospy.spin()

    def sub_cb(self, msg):
        for message in msg.Messages:
            [topic_type, topic_package] = self.extract_type_and_package(message.TopicType)
            full_topic_name = (self.namespace + "/" + message.TopicName).lstrip("/")
            if (full_topic_name in self.topic_pubs):
                try:
                    new_msg = eval(topic_type)()
                    new_msg.deserialize(message.SerializedMessageData)
                    self.topic_pubs[full_topic_name].publish(new_msg)
                except:
                    rospy.logerr("Could not deserialize/republish message on " + full_topic_name + " of type " + message.TopicType + " using existing publisher")
            elif (topic_package in self.imported_packages):
                try:
                    self.topic_pubs[full_topic_name] = rospy.Publisher(full_topic_name, eval(topic_type))
                    new_msg = eval(topic_type)()
                    new_msg.deserialize(message.SerializedMessageData)
                    self.topic_pubs[full_topic_name].publish(new_msg)
                except:
                    rospy.logerr("Could not deserialize/republish message on " + full_topic_name + " of type " + message.TopicType + " using new publisher")
            else:
                try:
                    self.dynamic_load(topic_package)
                    self.imported_packages.append(topic_package)
                    self.topic_pubs[full_topic_name] = rospy.Publisher(full_topic_name, eval(topic_type))
                    new_msg = eval(topic_type)()
                    new_msg.deserialize(message.SerializedMessageData)
                    self.topic_pubs[full_topic_name].publish(new_msg)
                except:
                    rospy.logerr("Could not import/deserialize/republish message on " + full_topic_name + " of type " + message.TopicType + " using new publisher")

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('message_demultiplexer')
    aggregated_topic = rospy.get_param("~aggregated_topic", "test/Aggregated")
    republish_namespace = rospy.get_param("~republish_namespace", "workstation")
    MessageDemultiplexer(aggregated_topic, republish_namespace)
