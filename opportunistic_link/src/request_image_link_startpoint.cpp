#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/RequestImage.h>
#include <opportunistic_link/image_compression.h>

class RequestImageLinkStartpoint
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::ServiceServer data_server_;
    sensor_msgs::ImageConstPtr last_image_;
    image_compression::ImageHandler compressor_;

public:

    RequestImageLinkStartpoint(ros::NodeHandle &n, std::string image_topic, std::string data_service) : nh_(n), it_(n)
    {
        image_sub_ = it_.subscribe(image_topic, 1, &RequestImageLinkStartpoint::image_cb, this);
        data_server_ = nh_.advertiseService(data_service, &RequestImageLinkStartpoint::data_cb, this);
        std::string transport_in = image_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~RequestImageLinkStartpoint()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    bool data_cb(teleop_msgs::RequestImage::Request& req, teleop_msgs::RequestImage::Response& res)
    {
        if (last_image_)
        {
            res.available = true;
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
            ROS_DEBUG("Compression of %f %% took %f seconds", ratio, secs);
            ROS_DEBUG("Original size: %f KB - Compressed size: %f KB", ((float)last_image_->data.size() / 1000.0), ((float)res.image.data.size() / 1000.0));
            // Clear the cache
            last_image_ = sensor_msgs::ImageConstPtr();
            ROS_INFO("Responded with a single message");
        }
        else
        {
            res.available = false;
            ROS_WARN("Single message requested, none available");
        }
        return true;
    }

    void image_cb(const sensor_msgs::ImageConstPtr& image)
    {
        last_image_ = sensor_msgs::ImageConstPtr(image);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "request_image_link_startpoint");
    ROS_INFO("Starting request image link startpoint...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string image_topic;
    std::string data_service;
    nhp.param(std::string("image_topic"), image_topic, std::string("camera/rgb/image"));
    nhp.param(std::string("data_service"), data_service, std::string("camera/rgb/data"));
    RequestImageLinkStartpoint startpoint(nh, image_topic, data_service);
    ROS_INFO("...startup complete");
    startpoint.loop();
    return 0;
}
