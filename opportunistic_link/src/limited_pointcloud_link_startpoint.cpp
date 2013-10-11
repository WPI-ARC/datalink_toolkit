#include <ros/ros.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>
#include <sensor_msgs/PointCloud2.h>
#include <teleop_msgs/CompressedPointCloud2.h>
#include <zlib.h>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ros/conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>

void dealocate_sm_fn(sensor_msgs::PointCloud2 *p)
{
}

void dealocate_pcl_fn(pcl::PointCloud<pcl::PointXYZRGB> *p)
{
}

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
    std::vector<teleop_msgs::CompressedPointCloud2> pointclouds_;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_filter_;
    pcl::octree::PointCloudCompression<pcl::PointXYZRGB> encoder_;

public:

    Pointcloud2LinkStartpoint(ros::NodeHandle &n, double default_rate, uint8_t compression_type, float filter_size, std::string pointcloud_topic, std::string link_topic, std::string link_ctrl_service, std::string rate_ctrl_service) : nh_(n), repub_rate_(20.0)
    {
        forward_ = false;
        filter_size_ = filter_size;
        if (!((filter_size_ != INFINITY) && (filter_size_ >= 0.0) && (isnan(filter_size_) == false)))
        {
            filter_size_ = 0.0;
        }
        forward_rate_ = default_rate;
        if ((forward_rate_ != INFINITY) && (forward_rate_ > 0.0) && (isnan(forward_rate_) == false))
        {
            repub_rate_ = ros::Rate(forward_rate_);
        }
        compressed_pub_ = nh_.advertise<teleop_msgs::CompressedPointCloud2>(link_topic, 1, true);
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic, 1, &Pointcloud2LinkStartpoint::pointcloud_cb, this);
        link_server_ = nh_.advertiseService(link_ctrl_service, &Pointcloud2LinkStartpoint::link_control_cb, this);
        rate_server_ = nh_.advertiseService(rate_ctrl_service, &Pointcloud2LinkStartpoint::rate_control_cb, this);
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

    ~Pointcloud2LinkStartpoint()
    {
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
            ROS_INFO("Camera republishing paused");
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
            sensor_msgs::PointCloud2 intermediate;
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            if (filter_size_ > 0.0)
            {
                sensor_msgs::PointCloud2ConstPtr cloudptr(&cloud, dealocate_sm_fn);
                voxel_filter_.setInputCloud(cloudptr);
                voxel_filter_.setLeafSize(filter_size_, filter_size_, filter_size_);
                voxel_filter_.filter(intermediate);
            }
            else
            {
                intermediate = cloud;
            }
            teleop_msgs::CompressedPointCloud2 compressed = compress_pointcloud2(intermediate);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)compressed.compressed_data.size() / (float)cloud.data.size()) * 100.0;
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

    teleop_msgs::CompressedPointCloud2 compress_pointcloud2(sensor_msgs::PointCloud2& cloud)
    {
        teleop_msgs::CompressedPointCloud2 compressed_cloud;
        compressed_cloud.header.stamp = cloud.header.stamp;
        compressed_cloud.header.frame_id = cloud.header.frame_id;
        compressed_cloud.is_dense = cloud.is_dense;
        compressed_cloud.is_bigendian = cloud.is_bigendian;
        compressed_cloud.fields = cloud.fields;
        compressed_cloud.height = cloud.height;
        compressed_cloud.width = cloud.width;
        compressed_cloud.point_step = cloud.point_step;
        compressed_cloud.row_step = cloud.row_step;
        if (compression_type_ == teleop_msgs::CompressedPointCloud2::ZLIB)
        {
            // Compress the pointcloud data using ZLIB's DEFLATE compression
            compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::ZLIB;
            z_stream strm;
            std::vector<uint8_t> buffer;
            const size_t BUFSIZE = 1024 * 1024;
            uint8_t temp_buffer[BUFSIZE];
            strm.zalloc = Z_NULL;
            strm.zfree = Z_NULL;
            strm.opaque = Z_NULL;
            strm.avail_in = cloud.data.size();
            strm.next_in = reinterpret_cast<uint8_t *>(cloud.data.data());
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
            int ret = deflateInit(&strm, Z_BEST_SPEED);
            if (ret != Z_OK)
            {
                (void)deflateEnd(&strm);
                ROS_ERROR("ZLIB unable to init deflate stream");
                throw std::invalid_argument("ZLIB unable to init deflate stream");
            }
            while (strm.avail_in != 0)
            {
                ret = deflate(&strm, Z_NO_FLUSH);
                if (ret != Z_OK)
                {
                    (void)deflateEnd(&strm);
                    ROS_ERROR("ZLIB unable to deflate stream");
                    throw std::invalid_argument("ZLIB unable to deflate stream");
                }
                if (strm.avail_out == 0)
                {
                    buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
                    strm.next_out = temp_buffer;
                    strm.avail_out = BUFSIZE;
                }
            }
            int deflate_ret = Z_OK;
            while (deflate_ret == Z_OK)
            {
                if (strm.avail_out == 0)
                {
                    buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
                    strm.next_out = temp_buffer;
                    strm.avail_out = BUFSIZE;
                }
                deflate_ret = deflate(&strm, Z_FINISH);
            }
            if (deflate_ret != Z_STREAM_END)
            {
                (void)deflateEnd(&strm);
                ROS_ERROR("ZLIB unable to deflate stream");
                throw std::invalid_argument("ZLIB unable to deflate stream");
            }
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
            (void)deflateEnd(&strm);
            compressed_cloud.compressed_data.swap(buffer);
        }
        else if (compression_type_ == teleop_msgs::CompressedPointCloud2::PCL)
        {
            compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::PCL;
            pcl::PointCloud<pcl::PointXYZRGB> converted_cloud;
            pcl::fromROSMsg(cloud, converted_cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudptr(&converted_cloud, dealocate_pcl_fn);
            std::stringstream compressed_data_stream;
            encoder_.encodePointCloud(cloudptr, compressed_data_stream);
            std::string compressed_data = compressed_data_stream.str();
            std::vector<uint8_t> buffer(compressed_data.begin(), compressed_data.end());
            compressed_cloud.compressed_data = buffer;
        }
        else
        {
            compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::NONE;
            compressed_cloud.compressed_data = cloud.data;
        }
        return compressed_cloud;
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
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/ctrl"));
    nhp.param(std::string("rate_ctrl"), rate_ctrl_service, std::string("camera/rate"));
    nhp.param(std::string("default_rate"), default_rate, (double)INFINITY);
    nhp.param(std::string("filter_size"), filter_size, 0.0);
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
    Pointcloud2LinkStartpoint startpoint(nh, default_rate, compression_id, filter_size, pointcloud_topic, link_topic, link_ctrl_service, rate_ctrl_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
