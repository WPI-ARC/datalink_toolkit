#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <datalink_msgs/LinkControl.h>
#include <datalink_msgs/RateControl.h>

#define DEFAULT_LOOP_RATE 128.0

void dealocate_info_fn(sensor_msgs::CameraInfo* info)
{
}

class CameraLinkEndpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Rate loop_rate_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber info_sub_;
    image_transport::CameraPublisher camera_pub_;
    ros::ServiceClient link_ctrl_client_;
    ros::Timer link_watchdog_;
    bool forward_;
    std::string image_topic_;
    std::string info_topic_;
    std::string camera_base_;
    std::string link_ctrl_service_;
    std::vector<sensor_msgs::Image> images_;
    std::vector<sensor_msgs::CameraInfo> infos_;

public:

    CameraLinkEndpoint(ros::NodeHandle &n, std::string image_topic, std::string info_topic, std::string camera_base, std::string link_ctrl_service, const double loop_rate, bool latched) : nh_(n), it_(n), loop_rate_(DEFAULT_LOOP_RATE)
    {
        if ((loop_rate != INFINITY) && (loop_rate > 0.0) && (isnan(loop_rate) == false))
        {
            loop_rate_ = ros::Rate(loop_rate);
        }
        else
        {
            ROS_ERROR("Invalid loop rate %f, setting to default %f", loop_rate, DEFAULT_LOOP_RATE);
        }
        image_topic_ = image_topic;
        info_topic_ = info_topic;
        camera_base_ = camera_base;
        forward_ = false;
        link_ctrl_service_ = link_ctrl_service;
        link_ctrl_client_ = nh_.serviceClient<datalink_msgs::LinkControl>(link_ctrl_service_);
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &CameraLinkEndpoint::link_watchdog_cb, this, true);
        image_transport::SubscriberStatusCallback camera_image_cb = boost::bind(&CameraLinkEndpoint::image_cb, this);
        ros::SubscriberStatusCallback camera_info_cb = boost::bind(&CameraLinkEndpoint::info_cb, this);
        camera_pub_ = it_.advertiseCamera(camera_base_, 1, camera_image_cb, camera_image_cb, camera_info_cb, camera_info_cb, ros::VoidPtr(), latched);
        image_sub_ = it_.subscribe(image_topic_, 1, &CameraLinkEndpoint::image_data_cb, this);
        info_sub_ = nh_.subscribe(info_topic_, 1, &CameraLinkEndpoint::info_data_cb, this);
        std::string transport_in = image_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    void loop()
    {
        while (ros::ok())
        {
            loop_rate_.sleep();
            ros::spinOnce();
        }
    }

    void image_data_cb(const sensor_msgs::ImageConstPtr& imageptr)
    {
        sensor_msgs::Image image = *imageptr;
        if (infos_.size() > 0)
        {
            sensor_msgs::CameraInfo info = infos_[0];
            info.header.stamp = image.header.stamp;
            camera_pub_.publish(image, info);
            link_watchdog_.stop();
            link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &CameraLinkEndpoint::link_watchdog_cb, this, true);
        }
        else
        {
            images_.clear();
            images_.push_back(image);
        }
    }

    void info_data_cb(sensor_msgs::CameraInfo info)
    {
        if (images_.size() > 0)
        {
            sensor_msgs::Image image = images_[0];
            info.header.stamp = image.header.stamp;
            camera_pub_.publish(image, info);
            images_.clear();
            link_watchdog_.stop();
            link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &CameraLinkEndpoint::link_watchdog_cb, this, true);
        }
        infos_.clear();
        infos_.push_back(info);
    }

    void image_cb()
    {
        uint32_t camera_subs = camera_pub_.getNumSubscribers();
        ROS_DEBUG("%ud current subscribers [image]", camera_subs);
        if (camera_subs == 1)
        {
            datalink_msgs::LinkControl::Request req;
            datalink_msgs::LinkControl::Response res;
            req.Forward = true;
            if (link_ctrl_client_.call(req, res) && res.State)
            {
                forward_ = res.State;
                ROS_INFO("Turned forwarding on");
            }
            else
            {
                ROS_ERROR("Failed to contact link startpoint");
            }
        }
        else if (camera_subs < 1)
        {
            datalink_msgs::LinkControl::Request req;
            datalink_msgs::LinkControl::Response res;
            req.Forward = false;
            if (link_ctrl_client_.call(req, res) && !res.State)
            {
                forward_ = res.State;
                ROS_INFO("Turned forwarding off");
            }
            else
            {
                ROS_ERROR("Failed to contact link startpoint");
            }
        }
    }

    void info_cb()
    {
        uint32_t camera_subs = camera_pub_.getNumSubscribers();
        ROS_DEBUG("%ud current subscribers [info]", camera_subs);
    }

    void link_watchdog_cb(const ros::TimerEvent &e)
    {
        try
        {
            image_sub_.shutdown();
            info_sub_.shutdown();
            link_ctrl_client_.shutdown();
            ROS_INFO("Closed existing service and subscriber");
        }
        catch (...)
        {
            ROS_WARN("Could not close existing service and subscriber");
        }
        try
        {
            ROS_INFO("Attempting to reconnect");
            image_sub_ = it_.subscribe(image_topic_, 1, &CameraLinkEndpoint::image_data_cb, this);
            info_sub_ = nh_.subscribe(info_topic_, 1, &CameraLinkEndpoint::info_data_cb, this);
            link_ctrl_client_ = nh_.serviceClient<datalink_msgs::LinkControl>(link_ctrl_service_);
            datalink_msgs::LinkControl::Request req;
            datalink_msgs::LinkControl::Response res;
            req.Forward = forward_;
            if (link_ctrl_client_.call(req, res))
            {
                ROS_INFO("Contacted startpoint to reset link forwarding");
            }
            else
            {
                ROS_WARN("Could not contact startpoint to reset link forwarding");
            }
        }
        catch (...)
        {
            ROS_ERROR("Reconnect attempt failed");
        }
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &CameraLinkEndpoint::link_watchdog_cb, this, true);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_link_endpoint");
    ROS_INFO("Starting camera link endpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string image_topic;
    std::string info_topic;
    std::string camera_base_topic;
    std::string link_ctrl_service;
    double loop_rate = DEFAULT_LOOP_RATE;
    bool latched = false;
    nhp.param(std::string("link_image_topic"), image_topic, std::string("link/camera/rgb/image"));
    nhp.param(std::string("link_info_topic"), info_topic, std::string("link/camera/rgb/camera_info"));
    nhp.param(std::string("relay_base_topic"), camera_base_topic, std::string("relay/camera/rgb/image"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/rgb/ctrl"));
    nhp.param(std::string("loop_rate"), loop_rate, DEFAULT_LOOP_RATE);
    nhp.param(std::string("latched"), latched, false);
    CameraLinkEndpoint endpoint(nh, image_topic, info_topic, camera_base_topic, link_ctrl_service, loop_rate, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
