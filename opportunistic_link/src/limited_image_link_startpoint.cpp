#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>

class ImageLinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    bool forward_;
    double forward_rate_;
    ros::Rate repub_rate_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::ServiceServer link_server_;
    ros::ServiceServer rate_server_;
    sensor_msgs::ImageConstPtr last_image_;

public:

    ImageLinkStartpoint(ros::NodeHandle &n, double default_rate, std::string image_topic, std::string link_topic, std::string link_ctrl_service, std::string rate_ctrl_service) : nh_(n), it_(n), repub_rate_(20.0)
    {
        forward_ = false;
        forward_rate_ = default_rate;
        if ((forward_rate_ != INFINITY) && (forward_rate_ > 0.0) && (isnan(forward_rate_) == false))
        {
            repub_rate_ = ros::Rate(forward_rate_);
        }
        image_pub_ = it_.advertise(link_topic, 1, true);
        image_sub_ = it_.subscribe(image_topic, 1, &ImageLinkStartpoint::image_cb, this);
        link_server_ = nh_.advertiseService(link_ctrl_service, &ImageLinkStartpoint::link_control_cb, this);
        rate_server_ = nh_.advertiseService(rate_ctrl_service, &ImageLinkStartpoint::rate_control_cb, this);
        std::string transport_in = image_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~ImageLinkStartpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            if (forward_ && (forward_rate_ != INFINITY) && (forward_rate_ != 0.0))
            {
                if (last_image_)
                {
                    image_pub_.publish(last_image_);
                    last_image_ = sensor_msgs::ImageConstPtr();
                }
            }
            repub_rate_.sleep();
            ros::spinOnce();
        }
    }

    bool link_control_cb(teleop_msgs::LinkControl::Request& req, teleop_msgs::LinkControl::Response& res)
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

    bool rate_control_cb(teleop_msgs::RateControl::Request& req, teleop_msgs::RateControl::Response& res)
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
            if (last_image_)
            {
                image_pub_.publish(last_image_);
                last_image_ = sensor_msgs::ImageConstPtr();
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

    void image_cb(const sensor_msgs::ImageConstPtr& image)
    {
        if (forward_ && (forward_rate_ == INFINITY))
        {
            image_pub_.publish(image);
        }
        else
        {
            last_image_ = sensor_msgs::ImageConstPtr(image);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_link_startpoint");
    ROS_INFO("Starting image link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string image_topic;
    std::string link_topic;
    std::string link_ctrl_service;
    std::string rate_ctrl_service;
    double default_rate;
    nhp.param(std::string("image_topic"), image_topic, std::string("camera/image"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/image"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/ctrl"));
    nhp.param(std::string("rate_ctrl"), rate_ctrl_service, std::string("camera/rate"));
    nhp.param(std::string("default_rate"), default_rate, (double)INFINITY);
    ImageLinkStartpoint startpoint(nh, default_rate, image_topic, link_topic, link_ctrl_service, rate_ctrl_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
