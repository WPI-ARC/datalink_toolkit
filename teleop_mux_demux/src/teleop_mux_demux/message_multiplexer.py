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
import threading
from copy import deepcopy
import message_multiplexer

class MessageMulitplexer:

    def __init__(self, aggregation_topic_in, aggregated_topic_out, publication_rate, max_queue=10):
        self.queue = [None for index in range(max_queue)]
        self.cur_index = 0
        self.max_index = max_queue - 1
        self.queue_lock = threading.Lock()
        self.publisher = rospy.Publisher(aggregated_topic_out, AggregatedMessages)
        self.subscriber = rospy.Subscriber(aggregation_topic_in, SerializedMessage, self.sub_cb)
        rospy.loginfo("Loaded Multiplexer")
        rate = rospy.Rate(publication_rate)
        while not rospy.is_shutdown():
            self.queue_lock.acquire()
            self.send()
            self.queue_lock.release()
            rate.sleep()

    def send(self):
        if (self.cur_index == 0):
            return
        aggregate_message = AggregatedMessages()
        for index in range(len(self.cur_index)):
            aggregate_message.Messages.append(self.queue[index])
        aggregate_message.header.stamp = rospy.Time.now()
        self.publisher.publish(aggregate_message)
        self.cur_index = 0

    def sub_cb(self, msg):
        self.queue_lock.acquire()
        self.queue[self.cur_index] = msg
        if (self.cur_index < self.max_index):
            self.cur_index += 1
        else:
            self.cur_index += 1
            self.send()
        self.queue_lock.release()

if __name__ == '__main__':
    rospy.init_node('message_multiplexer')
    aggregation_topic = rospy.get_param("~aggregation_topic", "test/Aggregator")
    aggregated_topic = rospy.get_param("~aggregated_topic", "test/Aggregated")
    MessageMultiplexer(aggregation_topic, aggregated_topic)
