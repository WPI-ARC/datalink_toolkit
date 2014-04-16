#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <datalink_msgs/RequestCamera.h>
#include <opportunistic_link/image_compression.h>

class RequestCameraLinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::ServiceServer data_server_;
    image_transport::CameraSubscriber camera_sub_;
    sensor_msgs::ImageConstPtr last_image_;
    sensor_msgs::CameraInfoConstPtr last_info_;
    image_compression::ImageHandler compressor_;

public:

    RequestCameraLinkStartpoint(ros::NodeHandle &n, std::string camera_topic, std::string data_service) : nh_(n), it_(n)
    {
        camera_sub_ = it_.subscribeCamera(camera_topic, 1, &RequestCameraLinkStartpoint::camera_cb, this);
        data_server_ = nh_.advertiseService(data_service, &RequestCameraLinkStartpoint::data_cb, this);
        std::string transport_in = camera_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~RequestCameraLinkStartpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    bool data_cb(datalink_msgs::RequestCamera::Request& req, datalink_msgs::RequestCamera::Response& res)
    {
        if (last_image_ && last_info_)
        {
            res.available = true;
            res.info = *last_info_;
            // If necessary, reset encoder state
            if (req.reset_encoder)
            {
                compressor_.reset_encoder();
            }
            // Compress
            res.encoding = last_image_->encoding;
            struct timespec st, et;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
            res.image = compressor_.compress_image(*last_image_, req.quality);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
            float secs = (float)(et.tv_sec - st.tv_sec);
            secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
            float ratio = ((float)res.image.data.size() / (float)last_image_->data.size()) * 100.0;
            ROS_INFO("Compression of %f %% took %f seconds", ratio, secs);
            ROS_INFO("Original size: %f KB - Compressed size: %f KB", ((float)last_image_->data.size() / 1000.0), ((float)res.image.data.size() / 1000.0));
            // Clear the cache
            last_image_ = sensor_msgs::ImageConstPtr();
            last_info_ = sensor_msgs::CameraInfoConstPtr();
            ROS_INFO("Responded with a single message");
        }
        else
        {
            res.available = false;
            ROS_WARN("Single message requested, none available");
        }
        return true;
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        last_image_ = sensor_msgs::ImageConstPtr(image);
        last_info_ = sensor_msgs::CameraInfoConstPtr(info);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "request_camera_link_startpoint");
    ROS_INFO("Starting request camera link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string camera_topic;
    std::string data_service;
    nhp.param(std::string("camera_topic"), camera_topic, std::string("camera/rgb/image"));
    nhp.param(std::string("data_service"), data_service, std::string("camera/rgb/data"));
    RequestCameraLinkStartpoint startpoint(nh, camera_topic, data_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
