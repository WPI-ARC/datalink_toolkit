#include <ros/ros.h>
#include <time.h>
#include <datalink_msgs/RequestPointCloud2.h>
#include <datalink_msgs/RateControl.h>
#include <datalink_msgs/FilterControl.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <pointcloud_compression/pointcloud_compression.h>

class RequestPointcloud2LinkEndpoint
{
protected:

    ros::NodeHandle nh_;
    bool forward_;
    double forward_rate_;
    float filter_size_;
    ros::Rate repub_rate_;
    bool override_timestamps_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceClient data_client_;
    ros::ServiceServer filter_server_;
    ros::ServiceServer rate_server_;
    uint32_t pointcloud_subs_;
    pointcloud_compression::PointCloudHandler decompressor_;

public:

    RequestPointcloud2LinkEndpoint(ros::NodeHandle &n, std::string relay_topic, std::string data_service, std::string filter_ctrl_service, std::string rate_ctrl_service, float default_filter_size, double default_rate, bool override_timestamps, bool latched) : nh_(n), repub_rate_(20.0)
    {
        filter_size_ = default_filter_size;
        forward_ = false;
        forward_rate_ = default_rate;
        if ((forward_rate_ != INFINITY) && (forward_rate_ > 0.0) && (isnan(forward_rate_) == false))
        {
            repub_rate_ = ros::Rate(forward_rate_);
        }
        override_timestamps_ = override_timestamps;
        data_client_ = nh_.serviceClient<datalink_msgs::RequestPointCloud2>(data_service);
        rate_server_ = nh_.advertiseService(rate_ctrl_service, &RequestPointcloud2LinkEndpoint::rate_cb, this);
        filter_server_ = nh_.advertiseService(filter_ctrl_service, &RequestPointcloud2LinkEndpoint::filter_cb, this);
        ros::SubscriberStatusCallback pointcloud_sub_cb = boost::bind(&RequestPointcloud2LinkEndpoint::subscriber_cb, this);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(relay_topic, 1, pointcloud_sub_cb, pointcloud_sub_cb, ros::VoidPtr(), latched);
    }

    void loop()
    {
        while (ros::ok())
        {
            if (forward_ && (forward_rate_ == INFINITY))
            {
                ROS_INFO("Requesting a pointcloud");
                try
                {
                    get_pointcloud();
                }
                catch (...)
                {
                    ROS_WARN("Failure requesting a pointcloud, unable to republish");
                }
            }
            if (forward_ && (forward_rate_ != INFINITY) && (forward_rate_ != 0.0))
            {
                ROS_INFO("Requesting a pointcloud");
                try
                {
                    get_pointcloud();
                }
                catch (...)
                {
                    ROS_WARN("Failure requesting a pointcloud, unable to republish");
                }
                repub_rate_.sleep();
            }
            ros::spinOnce();
        }
    }

    void get_pointcloud()
    {
        datalink_msgs::RequestPointCloud2::Request req;
        datalink_msgs::RequestPointCloud2::Response res;
        // Set the filter size for pre-compression voxel filtering
        req.filter_size = filter_size_;
        if (data_client_.call(req, res))
        {
            if (res.available)
            {
                // Decompress pointcloud
                struct timespec st, et;
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
                sensor_msgs::PointCloud2 cloud = decompressor_.decompress_pointcloud2(res.cloud);
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
                float secs = (float)(et.tv_sec - st.tv_sec);
                secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
                float ratio = ((float)cloud.data.size() / (float)res.cloud.compressed_data.size()) * 100.0;
                ROS_INFO("Decompression of %f %% took %f seconds", ratio, secs);
                if (override_timestamps_)
                {
                    cloud.header.stamp = ros::Time::now();
                }
                pointcloud_pub_.publish(cloud);
            }
            else
            {
                ROS_WARN("Contacted startpoint, but no pointclouds available");
            }
        }
        else
        {
            ROS_ERROR("Unable to contact startpoint");
            throw std::invalid_argument("Unable to contact startpoint");
        }
    }

    bool filter_cb(datalink_msgs::FilterControl::Request& req, datalink_msgs::FilterControl::Response& res)
    {
        if (req.FilterSize >= 0.0)
        {
            filter_size_ = req.FilterSize;
            ROS_INFO("Set filter size to %f", filter_size_);
        }
        else if (req.FilterSize > 100)
        {
            filter_size_ = 0.0;
            ROS_WARN("Attempted to set filter size below zero - set to zero instead");
        }
        res.State = filter_size_;
        return true;
    }

    bool rate_cb(datalink_msgs::RateControl::Request& req, datalink_msgs::RateControl::Response& res)
    {
        if (req.Rate > 0.0 && (req.Rate != INFINITY) && (isnan(req.Rate) == false))
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
            try
            {
                get_pointcloud();
            }
            catch (...)
            {
                ROS_WARN("Single message requested, unable to republish");
            }
        }
        else if (req.Rate == 0.0 || req.Rate == -0.0)
        {
            forward_rate_ = 0.0;
            res.State = forward_rate_;
            ROS_INFO("Pointcloud republishing paused");
        }
        else
        {
            ROS_ERROR("Invalid publish rate requested");
        }
        res.State = forward_rate_;
        return true;
    }

    void subscriber_cb()
    {
        pointcloud_subs_ = pointcloud_pub_.getNumSubscribers();
        if (pointcloud_subs_ == 1)
        {
            forward_ = true;
            ROS_INFO("Turned forwarding on");
        }
        else if (pointcloud_subs_ < 1)
        {
            forward_ = false;
            ROS_INFO("Turned forwarding off");
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "request_pointcloud_link_endpoint");
    ROS_INFO("Starting request pointcloud link endpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string relay_topic;
    std::string data_service;
    std::string filter_ctrl_service;
    std::string rate_ctrl_service;
    bool latched;
    double default_rate;
    double default_filter_size;
    bool override_timestamps;
    nhp.param(std::string("relay_topic"), relay_topic, std::string("camera/relay/depth/points_xyzrgb"));
    nhp.param(std::string("data_service"), data_service, std::string("camera/depth/data"));
    nhp.param(std::string("filter_ctrl"), filter_ctrl_service, std::string("camera/depth/quality"));
    nhp.param(std::string("rate_ctrl"), rate_ctrl_service, std::string("camera/depth/rate"));
    nhp.param(std::string("latched"), latched, false);
    nhp.param(std::string("default_rate"), default_rate, (double)INFINITY);
    nhp.param(std::string("default_filter_size"), default_filter_size, 0.02);
    nhp.param(std::string("override_timestamps"), override_timestamps, true);
    RequestPointcloud2LinkEndpoint endpoint(nh, relay_topic, data_service, filter_ctrl_service, rate_ctrl_service, (float)default_filter_size, default_rate, override_timestamps, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
