#include <ros/ros.h>
#include <time.h>
#include <datalink_msgs/RequestPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <pointcloud_compression/pointcloud_compression.h>

#define DEFAULT_LOOP_RATE 128.0

class RequestPointcloud2LinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    uint8_t compression_type_;
    ros::Subscriber pointcloud_sub_;
    ros::ServiceServer data_server_;
    std::vector<sensor_msgs::PointCloud2> pointclouds_;
    pointcloud_compression::PointCloudHandler compressor_;

public:

    RequestPointcloud2LinkStartpoint(ros::NodeHandle &n, uint8_t compression_type, std::string pointcloud_topic, std::string data_service, const double loop_rate) : nh_(n), loop_rate_(DEFAULT_LOOP_RATE)
    {
        if ((loop_rate != INFINITY) && (loop_rate > 0.0) && (isnan(loop_rate) == false))
        {
            loop_rate_ = ros::Rate(loop_rate);
        }
        else
        {
            ROS_ERROR("Invalid loop rate %f, setting to default %f", loop_rate, DEFAULT_LOOP_RATE);
        }
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic, 1, &RequestPointcloud2LinkStartpoint::pointcloud_cb, this);
        data_server_ = nh_.advertiseService(data_service, &RequestPointcloud2LinkStartpoint::data_cb, this);
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
    }

    void loop()
    {
        while (ros::ok())
        {
            loop_rate_.sleep();
            ros::spinOnce();
        }
    }

    bool data_cb(datalink_msgs::RequestPointCloud2::Request& req, datalink_msgs::RequestPointCloud2::Response& res)
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
            res.cloud = compressor_.compress_pointcloud2(pointclouds_[0], compression_type_, req.filter_size);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)res.cloud.compressed_data.size() / (float)pointclouds_[0].data.size()) * 100.0;
            ROS_INFO("Compression of %f %% took %f seconds", ratio, secs);
            ROS_INFO("Original size: %f KB - Compressed size: %f KB", ((float)pointclouds_[0].data.size() / 1000.0), ((float)res.cloud.compressed_data.size() / 1000.0));
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
    double loop_rate = DEFAULT_LOOP_RATE;
    nhp.param(std::string("pointcloud_topic"), pointcloud_topic, std::string("camera/depth/points_xyzrgb"));
    nhp.param(std::string("compression_type"), compression_type, std::string("ZLIB"));
    nhp.param(std::string("data_service"), data_service, std::string("camera/depth/data"));
    nhp.param(std::string("loop_rate"), loop_rate, DEFAULT_LOOP_RATE);
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
    RequestPointcloud2LinkStartpoint startpoint(nh, compression_id, pointcloud_topic, data_service, loop_rate);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
