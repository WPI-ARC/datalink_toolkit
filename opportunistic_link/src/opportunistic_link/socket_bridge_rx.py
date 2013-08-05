#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Alternative ROS transport method over sockets   #
#                                                   #
#####################################################

import rospy
import socket
import struct

class SocketBridgeRX:

    def __init__(self, target, port, output_topic, output_topic_type, checktime, timeout, READ_LEN):
        [self.topic_type, self.topic_package] = self.extract_type_and_package(output_topic_type)
        self.dynamic_load(self.topic_package)
        self.real_topic_type = eval(self.topic_type)
        self.name = rospy.get_name()
        self.OK = True
        self.port = port
        self.target = target
        self.pub = rospy.Publisher(output_topic, self.real_topic_type)
        self.recover_connection()
        self.buffer = ""
        self.msg_len = -1
        self.checktime = checktime
        self.timeout = timeout
        self.timer = rospy.Timer(rospy.Duration(self.checktime), self.connection_check, oneshot=False)
        rospy.loginfo("Loaded socket bridge [RX]")
        while not rospy.is_shutdown():
            if (self.OK):
                new_data = ""
                try:
                    new_data = self.connection.recv(READ_LEN)
                except:
                    continue
                if (new_data == ''):
                    self.connection.close()
                    rospy.logerr("Socket connection broken")
                    self.OK = False
                    continue
                else:
                    self.buffer += new_data
                    self.buffer = self.attempt_deserialization(self.buffer)
            else:
                self.recover_connection()

    def connection_check(self, event):
        try:
            self.connection.send(self.name + " | OK")
            self.OK = True
        except:
            rospy.logerr("Socket write operation failed")
            self.OK = False

    def recover_connection(self):
        rospy.loginfo("Attempting to recover socket connection...")
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.connect((self.target, self.port))
            self.connection.setblocking(1)
            self.OK = True
            rospy.loginfo("...socket connection recovered")
        except:
            self.connection.close()
            rospy.logerr("...unable to recover socket connection. Trying again in 5 seconds...")
            rospy.sleep(5.0)

    def attempt_deserialization(self, data_buffer):
        while (self.msg_len < 0 and len(data_buffer) >= 4) or (self.msg_len > -1 and len(data_buffer) >= self.msg_len):
            if (self.msg_len < 0 and len(data_buffer) >= 4):
                (self.msg_len,) = struct.unpack('<I', data_buffer[0:4])
                data_buffer = data_buffer[4:]
            if (self.msg_len > -1 and len(data_buffer) >= self.msg_len):
                serialized_message = data_buffer[0:self.msg_len]
                self.clean_and_pub(serialized_message)
                data_buffer = data_buffer[self.msg_len:]
                self.msg_len = -1
        rospy.logdebug("Deserialized as much of the buffer as possible")
        return data_buffer

    def clean_and_pub(self, raw):
        try:
            new_msg = self.real_topic_type()
            new_msg.deserialize(raw)
            self.pub.publish(new_msg)
        except:
            rospy.logerr("Failed to deserialize and republish message!")

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
    checktime = 5.0
    timeout = 5.0
    SocketBridgeRX(target, port, output_topic, topic_type, checktime, timeout, msg_len)
