#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>

class ImageLinkEndpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    bool forward_;
    image_transport::CameraSubscriber image_sub_;
    image_transport::CameraPublisher image_pub_;
    ros::ServiceClient link_ctrl_client_;
    uint32_t image_subs_;
    uint32_t info_subs_;
    ros::Timer link_watchdog_;
    std::string link_base_topic_;
    std::string link_ctrl_service_;

public:

    ImageLinkEndpoint(ros::NodeHandle &n, std::string relay_base_topic, std::string link_base_topic, std::string link_ctrl_service, bool latched) : nh_(n), it_(n)
    {
        forward_ = false;
        link_base_topic_ = link_base_topic;
        link_ctrl_service_ = link_ctrl_service;
        image_sub_ = it_.subscribeCamera(link_base_topic, 1, &ImageLinkEndpoint::image_data_cb, this);
        link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service);
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &ImageLinkEndpoint::link_watchdog_cb, this, true);
        image_transport::SubscriberStatusCallback camera_image_cb = boost::bind(&ImageLinkEndpoint::image_cb, this);
        ros::SubscriberStatusCallback camera_info_cb = boost::bind(&ImageLinkEndpoint::info_cb, this);
        image_pub_ = it_.advertiseCamera(relay_base_topic, 1, camera_image_cb, camera_image_cb, camera_info_cb, camera_info_cb, ros::VoidPtr(), latched);
    }

    ~ImageLinkEndpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void image_data_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        image_pub_.publish(image, info);
        link_watchdog_.stop();
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &ImageLinkEndpoint::link_watchdog_cb, this, true);
    }

    void image_cb()
    {
        image_subs_ = image_pub_.getNumSubscribers();
        if (image_subs_ == 1)
        {
            teleop_msgs::LinkControl::Request req;
            teleop_msgs::LinkControl::Response res;
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
        else if (image_subs_ < 1)
        {
            teleop_msgs::LinkControl::Request req;
            teleop_msgs::LinkControl::Response res;
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
        info_subs_ = image_pub_.getNumSubscribers();
    }

    void link_watchdog_cb(const ros::TimerEvent &e)
    {
        teleop_msgs::LinkControl::Request req;
        teleop_msgs::LinkControl::Response res;
        req.Forward = forward_;
        if (link_ctrl_client_.call(req, res))
        {
            ROS_INFO("Link OK");
        }
        else
        {
            ROS_ERROR("Failed to contact link startpoint - attempting to reconnect");
            image_sub_ = it_.subscribeCamera(link_base_topic_, 1, &ImageLinkEndpoint::image_data_cb, this);
            link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service_);
        }
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &ImageLinkEndpoint::link_watchdog_cb, this, true);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_link_endpoint");
    ROS_INFO("Starting image link endpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string relay_base_topic;
    std::string link_base_topic;
    std::string link_ctrl_service;
    bool latched;
    nhp.param(std::string("relay_base_topic"), relay_base_topic, std::string("relay/camera/image"));
    nhp.param(std::string("link_base_topic"), link_base_topic, std::string("link/camera/image"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/ctrl"));
    nhp.param(std::string("latched"), latched, false);
    ImageLinkEndpoint endpoint(nh, relay_base_topic, link_base_topic, link_ctrl_service, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
