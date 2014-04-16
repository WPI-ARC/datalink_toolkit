#include <ros/ros.h>
#include <time.h>
#include <datalink_msgs/LinkControl.h>
#include <datalink_msgs/RateControl.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <pointcloud_compression/pointcloud_compression.h>

class Pointcloud2LinkEndpoint
{
protected:

    ros::NodeHandle nh_;
    bool forward_;
    bool override_timestamps_;
    ros::Publisher pointcloud_pub_;
    ros::Subscriber compressed_sub_;
    ros::ServiceClient link_ctrl_client_;
    uint32_t pointcloud_subs_;
    ros::Timer link_watchdog_;
    std::string link_topic_;
    std::string link_ctrl_service_;
    pointcloud_compression::PointCloudHandler decompressor_;

public:

    Pointcloud2LinkEndpoint(ros::NodeHandle &n, std::string relay_topic, std::string link_topic, std::string link_ctrl_service, bool override_timestamps, bool latched) : nh_(n)
    {
        forward_ = false;
        override_timestamps_ = override_timestamps;
        link_topic_ = link_topic;
        link_ctrl_service_ = link_ctrl_service;
        link_ctrl_client_ = nh_.serviceClient<datalink_msgs::LinkControl>(link_ctrl_service_);
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &Pointcloud2LinkEndpoint::link_watchdog_cb, this, true);
        ros::SubscriberStatusCallback pointcloud_sub_cb = boost::bind(&Pointcloud2LinkEndpoint::subscriber_cb, this);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(relay_topic, 1, pointcloud_sub_cb, pointcloud_sub_cb, ros::VoidPtr(), latched);
        compressed_sub_ = nh_.subscribe(link_topic_, 1, &Pointcloud2LinkEndpoint::pointcloud_data_cb, this);
    }

    ~Pointcloud2LinkEndpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void pointcloud_data_cb(datalink_msgs::CompressedPointCloud2 compressed)
    {
        try
        {
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            sensor_msgs::PointCloud2 cloud = decompressor_.decompress_pointcloud2(compressed);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)cloud.data.size() / (float)compressed.compressed_data.size()) * 100.0;
            ROS_DEBUG("Decompression of %f %% took %f seconds", ratio, secs);
            if (override_timestamps_)
            {
                cloud.header.stamp = ros::Time::now();
            }
            pointcloud_pub_.publish(cloud);
        }
        catch (...)
        {
            ROS_ERROR("Could not decompress pointcloud");
        }
        link_watchdog_.stop();
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &Pointcloud2LinkEndpoint::link_watchdog_cb, this, true);
    }

    void subscriber_cb()
    {
        pointcloud_subs_ = pointcloud_pub_.getNumSubscribers();
        if (pointcloud_subs_ == 1)
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
        else if (pointcloud_subs_ < 1)
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

    void link_watchdog_cb(const ros::TimerEvent &e)
    {
        try
        {
            compressed_sub_.shutdown();
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
            compressed_sub_ = nh_.subscribe(link_topic_, 1, &Pointcloud2LinkEndpoint::pointcloud_data_cb, this);
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
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &Pointcloud2LinkEndpoint::link_watchdog_cb, this, true);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_link_endpoint");
    ROS_INFO("Starting pointcloud link endpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string relay_topic;
    std::string link_topic;
    std::string link_ctrl_service;
    bool latched;
    bool override_timestamps;
    nhp.param(std::string("relay_topic"), relay_topic, std::string("relay/camera/depth/points_xyzrgb"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/depth/compressed"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/depth/ctrl"));
    nhp.param(std::string("latched"), latched, false);
    nhp.param(std::string("override_timestamps"), override_timestamps, true);
    Pointcloud2LinkEndpoint endpoint(nh, relay_topic, link_topic, link_ctrl_service, override_timestamps, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
