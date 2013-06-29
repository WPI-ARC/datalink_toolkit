#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   General purpose ROS message serializer          #
#                                                   #
#####################################################

import rospy
import StringIO
from std_msgs.msg import *
from telop_msgs.msg import *
import message_serializer

class MessageSerializer:

    def __init__(self, input_topic, input_topic_type, aggregation_topic):
        self.topic_name = input_topic
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.publisher = rospy.Publish(aggregation_topic, SerializedMessage)
        self.subscriber = rospy.Subscriber(input_topic, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("Loaded Serializer")
        while not rospy.is_shutdown():
            rospy.spin()

    def sub_cb(self, msg):
        buff = StringIO.StringIO()
        msg.serialize(buff)
        serialized_msg = SerializedMessage()
        serialized_msg.TopicName = self.topic_name
        serialized_msg.TopicType = self.topic_package + "/" + self.topic_type
        serialized_msg.SerializedMessageData = buff.getvalue()
        self.publisher.publish(serialized_msg)
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
    rospy.init_node('Serializer')
    topic_name = rospy.get_param("~topic_name", "test/Test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    aggregation_topic = rospy.get_param("~aggregation_topic", "test/Aggregated")
    MessageSerializer(topic_name, topic_type, aggregation_topic)
