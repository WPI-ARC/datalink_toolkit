#include <ros/ros.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>
#include <sensor_msgs/PointCloud2.h>
#include <teleop_msgs/CompressedPointCloud2.h>
#include <zlib.h>

class Pointcloud2LinkEndpoint
{
protected:

    ros::NodeHandle nh_;
    bool forward_;
    ros::Publisher pointcloud_pub_;
    ros::Subscriber compressed_sub_;
    ros::ServiceClient link_ctrl_client_;
    uint32_t pointcloud_subs_;
    ros::Timer link_watchdog_;
    std::string link_topic_;
    std::string link_ctrl_service_;

public:

    Pointcloud2LinkEndpoint(ros::NodeHandle &n, std::string relay_topic, std::string link_topic, std::string link_ctrl_service, bool latched) : nh_(n)
    {
        forward_ = false;
        link_topic_ = link_topic;
        link_ctrl_service_ = link_ctrl_service;
        link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service_);
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

    void pointcloud_data_cb(teleop_msgs::CompressedPointCloud2 compressed)
    {
        try
        {
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            sensor_msgs::PointCloud2 cloud = decompress_pointcloud2(compressed);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)cloud.data.size() / (float)compressed.compressed_data.size()) * 100.0;
            ROS_DEBUG("Decompression of %f %% took %f seconds", ratio, secs);
            pointcloud_pub_.publish(cloud);
        }
        catch (...)
        {
            ROS_ERROR("Could not decompress pointcloud");
        }
        link_watchdog_.stop();
        link_watchdog_ = nh_.createTimer(ros::Duration(10.0), &Pointcloud2LinkEndpoint::link_watchdog_cb, this, true);
    }

    sensor_msgs::PointCloud2 decompress_pointcloud2(teleop_msgs::CompressedPointCloud2& compressed)
    {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = compressed.header.stamp;
        cloud.header.frame_id = compressed.header.frame_id;
        cloud.is_dense = compressed.is_dense;
        cloud.is_bigendian = compressed.is_bigendian;
        cloud.fields = compressed.fields;
        cloud.height = compressed.height;
        cloud.width = compressed.width;
        cloud.point_step = compressed.point_step;
        cloud.row_step = compressed.row_step;
        if (compressed.compression_type == teleop_msgs::CompressedPointCloud2::ZLIB)
        {
            // Decompress the pointcloud data using ZLIB's DEFLATE compression
            z_stream strm;
            std::vector<uint8_t> buffer;
            const size_t BUFSIZE = 1024 * 1024;
            uint8_t temp_buffer[BUFSIZE];
            strm.zalloc = Z_NULL;
            strm.zfree = Z_NULL;
            strm.opaque = Z_NULL;
            int ret = inflateInit(&strm);
            if (ret != Z_OK)
            {
                (void)inflateEnd(&strm);
                ROS_ERROR("ZLIB unable to init inflate stream");
                throw std::invalid_argument("ZLIB unable to init inflate stream");
            }
            strm.avail_in = compressed.compressed_data.size();
            strm.next_in = reinterpret_cast<uint8_t *>(compressed.compressed_data.data());
            do
            {
                strm.next_out = temp_buffer;
                strm.avail_out = BUFSIZE;
                ret = inflate(&strm, Z_NO_FLUSH);
                if (buffer.size() < strm.total_out)
                {
                    buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
                }
            }
            while (ret == Z_OK);
            if (ret != Z_STREAM_END)
            {
                (void)inflateEnd(&strm);
                ROS_ERROR("ZLIB unable to inflate stream with ret=%d", ret);
                throw std::invalid_argument("ZLIB unable to inflate stream");
            }
            (void)inflateEnd(&strm);
            cloud.data.swap(buffer);
        }
        else if (compressed.compression_type == teleop_msgs::CompressedPointCloud2::PCL)
        {
            ;
        }
        else if (compressed.compression_type == teleop_msgs::CompressedPointCloud2::NONE)
        {
            cloud.data = compressed.compressed_data;
        }
        else
        {
            ROS_ERROR("Unsupported compression type");
            throw std::invalid_argument("Unsupported compression type");
        }
        return cloud;
    }

    void subscriber_cb()
    {
        pointcloud_subs_ = pointcloud_pub_.getNumSubscribers();
        if (pointcloud_subs_ == 1)
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
        else if (pointcloud_subs_ < 1)
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
            link_ctrl_client_ = nh_.serviceClient<teleop_msgs::LinkControl>(link_ctrl_service_);
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
    nhp.param(std::string("relay_topic"), relay_topic, std::string("relay/camera/depth/points_xyzrgb"));
    nhp.param(std::string("link_topic"), link_topic, std::string("link/camera/depth/compressed"));
    nhp.param(std::string("link_ctrl"), link_ctrl_service, std::string("camera/ctrl"));
    nhp.param(std::string("latched"), latched, false);
    Pointcloud2LinkEndpoint endpoint(nh, relay_topic, link_topic, link_ctrl_service, latched);
    ROS_INFO("...startup complete");
    endpoint.loop();
    return 0;
}
