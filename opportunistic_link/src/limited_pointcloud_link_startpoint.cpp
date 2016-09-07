#include <ros/ros.h>
#include <time.h>
#include <datalink_msgs/LinkControl.h>
#include <datalink_msgs/RateControl.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <pointcloud_compression/pointcloud_compression.h>

class Pointcloud2LinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    bool forward_;
    float filter_size_;
    double forward_rate_;
    uint8_t compression_type_;
    ros::Rate repub_rate_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher compressed_pub_;
    ros::ServiceServer link_server_;
    ros::ServiceServer rate_server_;
    std::vector<datalink_msgs::CompressedPointCloud2> pointclouds_;
    pointcloud_compression::PointCloudHandler compressor_;

public:

    Pointcloud2LinkStartpoint(ros::NodeHandle &n, double default_rate, uint8_t compression_type, float filter_size, std::string pointcloud_topic, std::string link_topic, std::string link_ctrl_service, std::string rate_ctrl_service) : nh_(n), repub_rate_(20.0)
    {
        forward_ = false;
        filter_size_ = filter_size;
        if (!((filter_size_ != INFINITY) && (filter_size_ >= 0.0) && (std::isnan(filter_size_) == false)))
        {
            filter_size_ = 0.0;
        }
        forward_rate_ = default_rate;
        if ((forward_rate_ != INFINITY) && (forward_rate_ > 0.0) && (std::isnan(forward_rate_) == false))
        {
            repub_rate_ = ros::Rate(forward_rate_);
        }
        compressed_pub_ = nh_.advertise<datalink_msgs::CompressedPointCloud2>(link_topic, 1, true);
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic, 1, &Pointcloud2LinkStartpoint::pointcloud_cb, this);
        link_server_ = nh_.advertiseService(link_ctrl_service, &Pointcloud2LinkStartpoint::link_control_cb, this);
        rate_server_ = nh_.advertiseService(rate_ctrl_service, &Pointcloud2LinkStartpoint::rate_control_cb, this);
        if (compression_type == datalink_msgs::CompressedPointCloud2::ZLIB)
        {
            compression_type_ = datalink_msgs::CompressedPointCloud2::ZLIB;
            ROS_INFO("Relay using ZLIB compression");
        }
        else if (compression_type == datalink_msgs::CompressedPointCloud2::PCL)
        {
            compression_type_ = datalink_msgs::CompressedPointCloud2::PCL;
            ROS_INFO("Relay using PCL compression");
        }
        else if (compression_type == datalink_msgs::CompressedPointCloud2::PC30)
        {
            compression_type_ = datalink_msgs::CompressedPointCloud2::PC30;
            ROS_INFO("Relay using PC30 compression");
        }
        else if (compression_type == datalink_msgs::CompressedPointCloud2::PC60)
        {
            compression_type_ = datalink_msgs::CompressedPointCloud2::PC60;
            ROS_INFO("Relay using PC60 compression");
        }
        else
        {
            compression_type_ = datalink_msgs::CompressedPointCloud2::NONE;
            ROS_WARN("Relay using NONE compression");
        }
        ROS_INFO("Relay using %f voxel filter size", filter_size_);
    }

    void loop()
    {
        while (ros::ok())
        {
            if (forward_ && (forward_rate_ != INFINITY) && (forward_rate_ != 0.0))
            {
                if (pointclouds_.size() > 0)
                {
                    compressed_pub_.publish(pointclouds_[0]);
                    pointclouds_.clear();
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
        if (req.Rate > 0.0 && (req.Rate != INFINITY) && (std::isnan(req.Rate) == false))
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
            if (pointclouds_.size() > 0)
            {
                compressed_pub_.publish(pointclouds_[0]);
                pointclouds_.clear();
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
            ROS_INFO("Pointcloud republishing paused");
        }
        else
        {
            ROS_ERROR("Invalid publish rate requested");
        }
        res.State = forward_rate_;
        return true;
    }

    void pointcloud_cb(sensor_msgs::PointCloud2 cloud)
    {
        try
        {
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            datalink_msgs::CompressedPointCloud2 compressed = compressor_.compress_pointcloud2(cloud, compression_type_, filter_size_);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            double secs = (double)(et.tv_sec - st.tv_sec);
            secs = secs + (double)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            double ratio = ((double)compressed.compressed_data.size() / (double)cloud.data.size()) * 100.0;
            ROS_DEBUG("Compression of %f %% took %f seconds", ratio, secs);
            ROS_DEBUG("Original size: %f KB - Compressed size: %f KB", ((float)cloud.data.size() / 1000.0), ((float)compressed.compressed_data.size() / 1000.0));
            if (forward_ && (forward_rate_ == INFINITY))
            {
                compressed_pub_.publish(compressed);
            }
            else
            {
                if (pointclouds_.size() > 0)
                {
                    pointclouds_.clear();
                }
                pointclouds_.push_back(compressed);
            }
        }
        catch (...)
        {
            ROS_ERROR("Could not compress pointcloud");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_link_startpoint");
    ROS_INFO("Starting pointcloud link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string pointcloud_topic;
    std::string compression_type;
    std::string link_topic;
    std::string link_ctrl_service;
    std::string rate_ctrl_service;
    double default_rate;
    double filter_size;
    nhp.param(std::string("pointcloud_topic"), pointcloud_topic, std::string("camera/depth/points_xyzrgb"));
    nhp.param(std::string("compression_type"), compression_type, std::string("NONE"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/depth/compressed"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/depth/ctrl"));
    nhp.param(std::string("rate_ctrl"), rate_ctrl_service, std::string("camera/depth/rate"));
    nhp.param(std::string("default_rate"), default_rate, (double)INFINITY);
    nhp.param(std::string("filter_size"), filter_size, 0.0);
    uint8_t compression_id;
    if (compression_type.compare("ZLIB") == 0)
    {
        compression_id = datalink_msgs::CompressedPointCloud2::ZLIB;
    }
    else if (compression_type.compare("PCL") == 0)
    {
        compression_id = datalink_msgs::CompressedPointCloud2::PCL;
    }
    else if (compression_type.compare("PC30") == 0)
    {
        compression_id = datalink_msgs::CompressedPointCloud2::PC30;
    }
    else if (compression_type.compare("PC60") == 0)
    {
        compression_id = datalink_msgs::CompressedPointCloud2::PC60;
    }
    else
    {
        compression_id = datalink_msgs::CompressedPointCloud2::NONE;
    }
    Pointcloud2LinkStartpoint startpoint(nh, default_rate, compression_id, (float)filter_size, pointcloud_topic, link_topic, link_ctrl_service, rate_ctrl_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
