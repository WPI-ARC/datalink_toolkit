#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <datalink_msgs/LinkControl.h>
#include <datalink_msgs/RateControl.h>

class CameraLinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    bool forward_;
    double forward_rate_;
    ros::Rate repub_rate_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::CameraPublisher camera_pub_;
    ros::ServiceServer link_server_;
    ros::ServiceServer rate_server_;
    sensor_msgs::ImageConstPtr last_image_;
    sensor_msgs::CameraInfoConstPtr last_info_;

public:

    CameraLinkStartpoint(ros::NodeHandle &n, double default_rate, std::string camera_base_topic, std::string link_base_topic, std::string link_ctrl_service, std::string rate_ctrl_service) : nh_(n), it_(n), repub_rate_(20.0)
    {
        forward_ = false;
        forward_rate_ = default_rate;
        if ((forward_rate_ != INFINITY) && (forward_rate_ > 0.0) && (isnan(forward_rate_) == false))
        {
            repub_rate_ = ros::Rate(forward_rate_);
        }
        camera_sub_ = it_.subscribeCamera(camera_base_topic, 1, &CameraLinkStartpoint::camera_cb, this);
        camera_pub_ = it_.advertiseCamera(link_base_topic, 1, true);
        link_server_ = nh_.advertiseService(link_ctrl_service, &CameraLinkStartpoint::link_control_cb, this);
        rate_server_ = nh_.advertiseService(rate_ctrl_service, &CameraLinkStartpoint::rate_control_cb, this);
        std::string transport_in = camera_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~CameraLinkStartpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            if (forward_ && (forward_rate_ != INFINITY) && (forward_rate_ != 0.0))
            {
                if (last_image_ && last_info_)
                {
                    camera_pub_.publish(last_image_, last_info_);
                    last_image_ = sensor_msgs::ImageConstPtr();
                    last_info_ = sensor_msgs::CameraInfoConstPtr();
                }
            }
            repub_rate_.sleep();
            ros::spinOnce();
        }
    }

    bool link_control_cb(datalink_msgs::LinkControl::Request& req, datalink_msgs::LinkControl::Response& res)
    {
        forward_ = req.Forward;
        res.State = forward_;
        if (forward_)
        {
            ROS_INFO("Set forwarding to TRUE");
        }
        else
        {
            ROS_INFO("Set forwarding to FALSE");
        }
        return true;
    }

    bool rate_control_cb(datalink_msgs::RateControl::Request& req, datalink_msgs::RateControl::Response& res)
    {
        if (req.Rate > 0.0 && (req.Rate != INFINITY) && (req.Rate != NAN))
        {
            repub_rate_ = ros::Rate(req.Rate);
            forward_rate_ = req.Rate;
            res.State = forward_rate_;
            ROS_INFO("Set republish rate to %f", forward_rate_);
        }
        else if (req.Rate == INFINITY)
        {
            forward_rate_ = INFINITY;
            ROS_INFO("Set republish rate to native");
        }
        else if ((req.Rate == -1.0) && (forward_rate_ != INFINITY))
        {
            if (last_image_ && last_info_)
            {
                camera_pub_.publish(last_image_, last_info_);
                last_image_ = sensor_msgs::ImageConstPtr();
                last_info_ = sensor_msgs::CameraInfoConstPtr();
                ROS_INFO("Single message republished");
            }
            else
            {
                ROS_WARN("Single message requested, none available");
            }
        }
        else if (req.Rate == 0.0 || req.Rate == -0.0)
        {
            forward_rate_ = 0.0;
            res.State = forward_rate_;
            ROS_INFO("Camera republishing paused");
        }
        else
        {
            ROS_ERROR("Invalid publish rate requested");
        }
        res.State = forward_rate_;
        return true;
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (forward_ && (forward_rate_ == INFINITY))
        {
            camera_pub_.publish(image, info);
        }
        else
        {
            last_image_ = sensor_msgs::ImageConstPtr(image);
            last_info_ = sensor_msgs::CameraInfoConstPtr(info);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_link_startpoint");
    ROS_INFO("Starting camera link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string camera_topic;
    std::string link_topic;
    std::string link_ctrl_service;
    std::string rate_ctrl_service;
    double default_rate;
    nhp.param(std::string("camera_topic"), camera_topic, std::string("camera/rgb/image"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/rgb/image"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/rgb/ctrl"));
    nhp.param(std::string("rate_ctrl"), rate_ctrl_service, std::string("camera/rgb/rate"));
    nhp.param(std::string("default_rate"), default_rate, (double)INFINITY);
    CameraLinkStartpoint startpoint(nh, default_rate, camera_topic, link_topic, link_ctrl_service, rate_ctrl_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
