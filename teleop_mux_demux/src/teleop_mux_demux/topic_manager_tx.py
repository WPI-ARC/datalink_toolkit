#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Experimental ROS Python adaptive topic handler  #
#   that dynamically adds topic handlers for new    #
#   topics that appear                              #
#                                                   #
#####################################################

import rospy
from std_msgs.msg import *
from teleop_msgs.msg import *
from teleop_msgs.srv import *
import StringIO

import topic_manager_tx

class TopicHandlerTX:

    def __init__(self, topic_name, real_topic_type, full_topic_type, aggregation_topic):
        self.full_topic_type = full_topic_type
        self.topic_name = topic_name
        self.forward = False
        self.rate = float('infinity')
        self.last_time = rospy.get_time()
        self.time_step = rospy.Duration(0.0)
        self.link_server = rospy.Service(topic_name + "/link_control", LinkControl, self.link_cb)
        self.rate_server = rospy.Service(topic_name + "/rate_control", RateControl, self.rate_cb)
        self.publisher = rospy.Publisher(aggregation_topic, SerializedMessage)
        self.subscriber = rospy.Subscriber(topic_name, real_topic_type, self.sub_cb)
        rospy.loginfo("Started topic handler - topic: " + self.topic_name)

    def link_cb(self, request):
        if (request.Forward == True):
            self.forward = True
            rospy.loginfo("Set forwarding to TRUE - topic: " + self.topic_name)
        elif (request.Forward == False):
            self.forward = False
            rospy.loginfo("Set forwarding to FALSE - topic: " + self.topic_name)
        response = LinkControlResponse()
        response.State = self.forward
        return response

    def rate_cb(self, request):
        self.rate = request.Rate
        self.time_step = rospy.Duration(1.0 / self.rate)
        rospy.loginfo("Set rate to " + str(self.rate) + " - topic: " + self.topic_name)
        response = RateControlResponse()
        response.State = self.rate
        return response

    def sub_cb(self, msg):
        if (self.forward and (self.rate == float('infinity'))):
            buff = StringIO.StringIO()
            msg.serialize(buff)
            serialized_msg = SerializedMessage()
            serialized_msg.TopicName = self.topic_name
            serialized_msg.TopicType = self.full_topic_type
            serialized_msg.SerializedMessageData = buff.getvalue()
            buff.close()
            self.publisher.publish(serialized_msg)
        elif (self.forward):
            new_time = rospy.get_time()
            if (rospy.Duration(new_time - self.last_time) > self.time_step):
                self.last_time = new_time
                buff = StringIO.StringIO()
                msg.serialize(buff)
                serialized_msg = SerializedMessage()
                serialized_msg.TopicName = self.topic_name
                serialized_msg.TopicType = self.full_topic_type
                serialized_msg.SerializedMessageData = buff.getvalue()
                buff.close()
                self.publisher.publish(serialized_msg)

class TopicManagerTX:

    def __init__(self, namespace, aggregation_topic, topic_blacklist=['rosout', 'rosout_agg', 'Aggregation', 'Aggregated'], topic_whitelist=[], check_rate=1):
        self.namespace = namespace
        self.aggregation_topic = aggregation_topic
        self.imported_packages = ["std_msgs", "teleop_msgs"]
        self.available_topics = {}
        self.blacklisted_topics = topic_blacklist
        self.whitelisted_topics = topic_whitelist
        self.topic_handlers = {}
        self.update_publisher = rospy.Publisher(aggregation_topic, SerializedMessage)
        self.master = rospy.get_master()
        rate = rospy.Rate(check_rate)
        rospy.loginfo("Loaded topic manager [TX]")
        while not rospy.is_shutdown():
            self.update_available_topics()
            rate.sleep()

    def update_available_topics(self):
        [code, msg, values] = self.master.getPublishedTopics(self.namespace)
        if (code != 1):
            rospy.logerr("Unable to check published topics - error message: " + msg + "!")
        else:
            system_update = TopicUpdate()
            for topic_info in values:
                topic_name = topic_info[0].lstrip("/")
                try:
                    existing_type = self.available_topics[topic_name]
                    if (existing_type != topic_info[1]):
                        rospy.logerr("Already watching topic but the type has changed!")
                        self.available_topics[topic_name] = topic_info[1]
                        self.topic_handlers[topic_name] = self.build_new_handler(topic_name, topic_info[1], self.aggregation_topic)
                    system_update.TopicNames.append(topic_name)
                    system_update.TopicTypes.append(topic_info[1])
                except:
                    if ((self.whitelisted_topics == []) and not self.is_blacklisted(topic_name)):
                        rospy.loginfo("Adding subscriber to a newly available topic: " + topic_name)
                        self.available_topics[topic_name] = topic_info[1]
                        self.topic_handlers[topic_name] = self.build_new_handler(topic_name, topic_info[1], self.aggregation_topic)
                        system_update.TopicNames.append(topic_name)
                        system_update.TopicTypes.append(topic_info[1])
                    elif (topic_name in self.whitelisted_topics):
                        rospy.loginfo("Adding subscriber to a newly available topic: " + topic_name)
                        self.available_topics[topic_name] = topic_info[1]
                        self.topic_handlers[topic_name] = self.build_new_handler(topic_name, topic_info[1], self.aggregation_topic)
                        system_update.TopicNames.append(topic_name)
                        system_update.TopicTypes.append(topic_info[1])
            # Package the update message (instead of actually publishing it, we skip
            # that step and directly serialize it for aggregation as there are no
            # subscribers for it on this side of the link
            system_update.header.stamp = rospy.Time.now()
            buff = StringIO.StringIO()
            system_update.serialize(buff)
            serialized_msg = SerializedMessage()
            serialized_msg.TopicName = "Updates"
            serialized_msg.TopicType = "teleop_msgs/TopicUpdate"
            serialized_msg.SerializedMessageData = buff.getvalue()
            buff.close()
            self.update_publisher.publish(serialized_msg)

    def is_blacklisted(self, potential_topic):
        for entry in self.blacklisted_topics:
            if (potential_topic == entry):
                return True
            if ('*' in entry):
                namespace = entry.split("/*")[0]
                if (namespace in potential_topic):
                    return True
        return False

    def build_new_handler(self, topic_name, full_topic_type, aggregation_topic):
        [topic_type, topic_package] = self.extract_type_and_package(full_topic_type)
        if (topic_package not in self.imported_packages):
            self.dynamic_load(topic_package)
            self.imported_packages.append(topic_package)
        return TopicHandlerTX(topic_name, eval(topic_type), full_topic_type, aggregation_topic)

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('topic_handler_tx')
    aggregation_topic = rospy.get_param("~aggregation_topic", "Aggregation")
    TopicManagerTX(rospy.get_namespace(), aggregation_topic, ['rosout', 'rosout_agg', 'Aggregation', 'Aggregated', 'workstation/*'])
