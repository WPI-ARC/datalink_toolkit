#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

class CameraResizer
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    int resized_width_;
    int resized_height_;
    bool convert_to_bw_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::CameraPublisher camera_pub_;

public:

    CameraResizer(ros::NodeHandle &n, std::string camera_base_topic, std::string resized_base_topic, int resized_width, int resized_height, bool convert_to_bw) : nh_(n), it_(n)
    {
        resized_width_ = resized_width;
        resized_height_ = resized_height;
        convert_to_bw_ = convert_to_bw;
        camera_sub_ = it_.subscribeCamera(camera_base_topic, 1, &CameraResizer::camera_cb, this);
        camera_pub_ = it_.advertiseCamera(resized_base_topic, 1, true);
        std::string transport_in = camera_sub_.getTransport();
        ROS_INFO("Actual image data topic: %s", camera_sub_.getTopic().c_str());
        ROS_INFO("Actual camera_info topic: %s", camera_sub_.getTopic().c_str());
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~CameraResizer()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        // Convert to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Make destination
        cv::Mat resized;
        if (convert_to_bw_)
        {
            // Drop to 1-channel
            resized = cv::Mat(cv::Size(resized_width_, resized_height_), CV_8UC1);
        }
        else
        {
            // Keep RGB channels
            resized = cv::Mat(cv::Size(resized_width_, resized_height_), CV_8UC3);
        }
        // Remove color if desired
        cv::Mat intermediate;
        if (convert_to_bw_)
        {
            // Convery to grayscale
            cv::cvtColor(cv_ptr->image, intermediate, CV_BGR2GRAY);
        }
        else
        {
            // Keep RGB
            intermediate = cv_ptr->image;
        }
        // Resize image
        if (resized_width_ < image->width && resized_height_ < image->height)
        {
            // If we're resizing smaller, use CV_INTER_AREA interpolation
            cv::resize(intermediate, resized, resized.size(), 0.0, 0.0, CV_INTER_AREA);
        }
        else
        {
            // If we're resizing bigger, use CV_INTER_LINEAR interpolation
            cv::resize(intermediate, resized, resized.size(), 0.0, 0.0, CV_INTER_LINEAR);
        }
        // Convert back to ROS
        sensor_msgs::Image resized_image;
        cv_bridge::CvImage converted;
        if (convert_to_bw_)
        {
            converted = cv_bridge::CvImage(image->header, "mono8", resized);
        }
        else
        {
            converted = cv_bridge::CvImage(image->header, image->encoding, resized);
        }
        converted.toImageMsg(resized_image);
        sensor_msgs::CameraInfo resized_info(*info);
        // Republish
        camera_pub_.publish(resized_image, resized_info);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_resizer");
    ROS_INFO("Starting camera resizer...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string camera_base_topic;
    std::string resized_base_topic;
    int resized_width;
    int resized_height;
    bool convert_to_bw;
    nhp.param(std::string("camera_base_topic"), camera_base_topic, std::string("camera/rgb/image"));
    nhp.param(std::string("resized_base_topic"), resized_base_topic, std::string("camera/resized/rgb/image"));
    nhp.param(std::string("resized_width"), resized_width, 160);
    nhp.param(std::string("resized_height"), resized_height, 120);
    nhp.param(std::string("convert_to_bw"), convert_to_bw, false);
    CameraResizer resizer(nh, camera_base_topic, resized_base_topic, resized_width, resized_height, convert_to_bw);
    ROS_INFO("...startup complete");
    resizer.loop();
    return 0;
}

