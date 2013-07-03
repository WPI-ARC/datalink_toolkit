#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Alternative ROS transport method over serial    #
#                                                   #
#####################################################

import rospy
import socket

class SocketBridgeRX:

    def __init__(self, target, port, output_topic, output_topic_type, MSG_LEN):
        [self.topic_type, self.topic_package] = self.extract_type_and_package(output_topic_type)
        self.dynamic_load(self.topic_package)
        self.real_topic_type = eval(self.topic_type)
        self.port = port
        self.target = target
        self.pub = rospy.Publisher(output_topic, self.real_topic_type)
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.connect((target, port))
        self.buffer = ""
        rospy.loginfo("Loaded socket bridge [RX]")
        while not rospy.is_shutdown():
            chunk = self.connection.recv(MSG_LEN)
            if (chunk == ''):
                rospy.logfatal("Socket connection terminated. Exiting...")
                rospy.signal_shutdown("Socket connection ended")
            self.buffer += chunk
            sx = self.buffer.find("<sbtxfs>")
            ex = self.buffer.find("</sbtxe>")
            if (sx != -1 and ex != -1):
                raw_data = self.buffer[sx + 8:ex]
                self.clean_and_pub(raw_data)
                try:
                    self.buffer = self.buffer[ex + 8:]
                except:
                    self.buffer = ""

    def clean_and_pub(self, raw):
        try:
            new_msg = self.real_topic_type()
            new_msg.deserialize(raw)
            self.pub.publish(new_msg)
        except:
            rospy.logerr("Failed to deserializa and republish message!")

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('socket_bridge_rx')
    output_topic = rospy.get_param("~output_topic", "bridged/test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    target = rospy.get_param("~target", "localhost")
    port = rospy.get_param("~port", 9000)
    msg_len = rospy.get_param("~msg_len", 4096)
    SocketBridgeRX(target, port, output_topic, topic_type, msg_len)
