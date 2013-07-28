#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Alternative ROS transport method over sockets   #
#                                                   #
#####################################################

import rospy
import struct
import socket
import StringIO
import threading
from copy import deepcopy

class SocketBridgeTX:

    def __init__(self, port, input_topic, input_topic_type, MSG_LEN):
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.port = port
        self.MSG_LEN = MSG_LEN
        self.socket_lock = threading.Lock()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, self.server_socket.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR) | 1)
        self.server_socket.bind(('', self.port))
        self.server_socket.listen(5)
        self.sockets = []
        self.sub = rospy.Subscriber(input_topic, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("Loaded socket bridge [TX]")
        rospy.on_shutdown(self.cleanup)
        while not rospy.is_shutdown():
            (clientsocket, address) = self.server_socket.accept()
            self.sockets.append(clientsocket)
            rospy.loginfo("Added socket to address: " + str(address))

    def cleanup(self):
        rospy.loginfo("Shutting down sockets...")
        for socket in self.sockets:
            socket.close()
        self.server_socket.close()
        rospy.loginfo("...Shutting down now!")

    def sub_cb(self, msg):
        with self.socket_lock:
            # Serialize the message for transport
            buff = StringIO.StringIO()
            start = buff.tell()
            buff.seek(start + 4)
            msg.serialize(buff)
            end = buff.tell()
            buff.seek(start)
            size = end - start - 4
            buff.write(struct.pack('<I', size))
            buff.seek(end)
            data = buff.getvalue()
            buff.close()
            # Send the data over all the available sockets
            for available_socket in self.sockets:
                try:
                    total_sent = 0
                    while (total_sent < len(data)):
                        sent = available_socket.send(data[total_sent:])
                        if (sent == 0):
                            rospy.logerr("Socket connection broken")
                            available_socket.close()
                            self.sockets.remove(available_socket)
                            break
                        else:
                            total_sent += sent
                except:
                    try:
                        available_socket.close()
                        self.sockets.remove(available_socket)
                        rospy.logerr("Socket connection failed")
                    except:
                        rospy.logerr("Socket connection failed and unable to close socket")

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('socket_bridge_tx')
    input_topic = rospy.get_param("~input_topic", "test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    port = rospy.get_param("~port", 9000)
    msg_len = rospy.get_param("~msg_len", 4096)
    SocketBridgeTX(port, input_topic, topic_type, msg_len)
