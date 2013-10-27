#include <ros/ros.h>
#include <time.h>
#include <teleop_msgs/RequestPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <teleop_msgs/CompressedPointCloud2.h>
#include <opportunistic_link/pointcloud_compression.h>

class RequestPointcloud2LinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    float filter_size_;
    uint8_t compression_type_;
    ros::Subscriber pointcloud_sub_;
    ros::ServiceServer data_server_;
    std::vector<sensor_msgs::PointCloud2> pointclouds_;
    pointcloud_compression::PointCloudHandler compressor_;

public:

    RequestPointcloud2LinkStartpoint(ros::NodeHandle &n, uint8_t compression_type, float filter_size, std::string pointcloud_topic, std::string data_service) : nh_(n)
    {
        filter_size_ = filter_size;
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic, 1, &RequestPointcloud2LinkStartpoint::pointcloud_cb, this);
        data_server_ = nh_.advertiseService(data_service, &RequestPointcloud2LinkStartpoint::data_cb, this);
        if (compression_type == teleop_msgs::CompressedPointCloud2::ZLIB)
        {
            compression_type_ = teleop_msgs::CompressedPointCloud2::ZLIB;
            ROS_INFO("Relay using ZLIB compression");
        }
        else if (compression_type == teleop_msgs::CompressedPointCloud2::PCL)
        {
            compression_type_ = teleop_msgs::CompressedPointCloud2::PCL;
            ROS_INFO("Relay using PCL compression");
        }
        else
        {
            compression_type_ = teleop_msgs::CompressedPointCloud2::NONE;
            ROS_WARN("Relay using NONE compression");
        }
        ROS_INFO("Relay using %f voxel filter size", filter_size_);
    }

    ~RequestPointcloud2LinkStartpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    bool data_cb(teleop_msgs::RequestPointCloud2::Request& req, teleop_msgs::RequestPointCloud2::Response& res)
    {
        if (pointclouds_.size() > 0)
        {
            res.available = true;
            // If necessary, reset encoder state
            if (req.reset_encoder)
            {
                compressor_.reset_encoder();
            }
            // Compress
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            res.cloud = compressor_.compress_pointcloud2(pointclouds_[0], compression_type_, filter_size_);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)res.cloud.compressed_data.size() / (float)pointclouds_[0].data.size()) * 100.0;
            ROS_DEBUG("Compression of %f %% took %f seconds", ratio, secs);
            ROS_DEBUG("Original size: %f KB - Compressed size: %f KB", ((float)pointclouds_[0].data.size() / 1000.0), ((float)res.cloud.compressed_data.size() / 1000.0));
            // Clear the cache
            pointclouds_.clear();
            ROS_INFO("Responded with a single message");
        }
        else
        {
            res.available = false;
            ROS_WARN("Single message requested, none available");
        }
        return true;
    }

    void pointcloud_cb(sensor_msgs::PointCloud2 cloud)
    {
        pointclouds_.clear();
        pointclouds_.push_back(cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "request_pointcloud_link_startpoint");
    ROS_INFO("Starting request pointcloud link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string pointcloud_topic;
    std::string compression_type;
    std::string data_service;
    double filter_size;
    nhp.param(std::string("pointcloud_topic"), pointcloud_topic, std::string("camera/depth/points_xyzrgb"));
    nhp.param(std::string("compression_type"), compression_type, std::string("ZLIB"));
    nhp.param(std::string("data_service"), data_service, std::string("camera/depth/data"));
    nhp.param(std::string("filter_size"), filter_size, 0.02);
    uint8_t compression_id;
    if (compression_type.compare("ZLIB") == 0)
    {
        compression_id = teleop_msgs::CompressedPointCloud2::ZLIB;
    }
    else if (compression_type.compare("PCL") == 0)
    {
        compression_id = teleop_msgs::CompressedPointCloud2::PCL;
    }
    else
    {
        compression_id = teleop_msgs::CompressedPointCloud2::NONE;
    }
    RequestPointcloud2LinkStartpoint startpoint(nh, compression_id, filter_size, pointcloud_topic, data_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
