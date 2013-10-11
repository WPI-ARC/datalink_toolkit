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
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::ServiceClient link_ctrl_client_;
    uint32_t image_subs_;
    ros::Timer link_watchdog_;
    std::string link_topic_;
    std::string link_ctrl_service_;

public:

    ImageLinkEndpoint(ros::NodeHandle &n, std::string relay_topic, std::string link_topic, std::string link_ctrl_service, bool latched) : nh_(n), it_(n)
    {
        forward_ = false;
        link_topic_ = link_topic;
        link_ctrl_service_ = link_ctrl_service;
        link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service);
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &ImageLinkEndpoint::link_watchdog_cb, this, true);
        image_transport::SubscriberStatusCallback camera_image_cb = boost::bind(&ImageLinkEndpoint::image_cb, this);
        image_pub_ = it_.advertise(relay_topic, 1, camera_image_cb, camera_image_cb, ros::VoidPtr(), latched);
        image_sub_ = it_.subscribe(link_topic_, 1, &ImageLinkEndpoint::image_data_cb, this);
        std::string transport_in = image_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
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

    void image_data_cb(const sensor_msgs::ImageConstPtr& image)
    {
        image_pub_.publish(image);
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

    void link_watchdog_cb(const ros::TimerEvent &e)
    {
        try
        {
            image_sub_.shutdown();
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
            image_sub_ = it_.subscribe(link_topic_, 1, &ImageLinkEndpoint::image_data_cb, this);
            link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service_);
            teleop_msgs::LinkControl::Request req;
            teleop_msgs::LinkControl::Response res;
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
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &ImageLinkEndpoint::link_watchdog_cb, this, true);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_link_endpoint");
    ROS_INFO("Starting image link endpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string relay_topic;
    std::string link_topic;
    std::string link_ctrl_service;
    bool latched;
    nhp.param(std::string("relay_topic"), relay_topic, std::string("relay/camera/image"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/image"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/ctrl"));
    nhp.param(std::string("latched"), latched, false);
    ImageLinkEndpoint endpoint(nh, relay_topic, link_topic, link_ctrl_service, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
