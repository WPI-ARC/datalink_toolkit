#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <teleop_msgs/LinkControl.h>
#include <teleop_msgs/RateControl.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

class CameraListener
{
protected:

    image_transport::CameraSubscriber camera_sub_;

public:

    sensor_msgs::ImageConstPtr last_image_;
    sensor_msgs::CameraInfoConstPtr last_info_;

    CameraListener(image_transport::ImageTransport &it, std::string camera_base_topic)
    {
        camera_sub_ = it.subscribeCamera(camera_base_topic, 1, &CameraListener::camera_cb, this);
        std::string transport_in = camera_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    CameraListener()
    {
    }

    ~CameraListener()
    {
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        last_image_ = sensor_msgs::ImageConstPtr(image);
        last_info_ = sensor_msgs::CameraInfoConstPtr(info);
    }

};

class CameraManager
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    std::map<std::string, CameraListener> active_cameras_;
    ros::ServiceServer control_server_;
    int resized_width_;
    int resized_height_;
    bool convert_to_bw_;
    std::vector<std::string> active_camera_names_;
    std::string active_camera_name_;
    image_transport::CameraPublisher camera_pub_;

public:

    CameraManager(ros::NodeHandle &n) : nh_(n), it_(n)
    {
        ros::NodeHandle pn("~");
        std::string managed_camera_topic;
        if (!pn.getParam(std::string("managed_base_topic"), managed_camera_topic))
        {
            ROS_FATAL("No output topic given. (namespace: %s)", pn.getNamespace().c_str());
            exit(1);
        }
        camera_pub_ = it_.advertiseCamera(managed_camera_topic, 1, true);
        XmlRpc::XmlRpcValue camera_names;
        if (!pn.getParam("cameras", camera_names))
        {
            ROS_FATAL("No cameras given. (namespace: %s)", pn.getNamespace().c_str());
            exit(1);
        }
        if (camera_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("Malformed camera specification.  (namespace: %s)", pn.getNamespace().c_str());
            exit(1);
        }
        // Get the real camera names
        for (size_t i = 0; i < camera_names.size(); ++i)
        {
            XmlRpc::XmlRpcValue &name_value = camera_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_FATAL("Array of camera names should contain all strings.  (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            active_camera_names_.push_back((std::string)name_value);
        }
        // Get the real topic names and build listeners
        for (size_t i = 0; i < active_camera_names_.size(); ++i)
        {
            std::string ns = std::string("topics/") + active_camera_names_[i];
            std::string base_topic;
            pn.param(ns + "/base_topic", base_topic, std::string(""));
            if (base_topic == "")
            {
                ROS_FATAL("Invalid camera name->topic mapping for camera %s", active_camera_names_[i].c_str());
                exit(1);
            }
            active_cameras_[active_camera_names_[i]] = CameraListener(it_, base_topic);
        }
        // Get and set the default camera
        std::string default_camera;
        if (!pn.getParam("default_camera", default_camera))
        {
            ROS_WARN("No camera selected by default, using camera zero instead");
            if (active_camera_names_.size() > 0)
            {
                active_camera_name_ = active_camera_names_[0];
            }
            else
            {
                ROS_FATAL("No cameras provided");
                exit(1);
            }
        }
        else
        {
            if (active_camera_names_.size() > 0)
            {
                for (size_t i = 0; i < active_camera_names_.size(); i++)
                {
                    if (active_camera_names_[i] == default_camera)
                    {
                        active_camera_name_ = default_camera;
                        break;
                    }
                }
                if (active_camera_name_ == "")
                {
                    ROS_ERROR("Invalid camera selected by default, using camera zero instead");
                    active_camera_name_ = active_camera_names_[0];
                }
            }
            else
            {
                ROS_FATAL("No cameras provided");
                exit(1);
            }
        }
    }

    ~CameraManager()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            CameraListener& cur_listener = active_cameras_[active_camera_name_];
//            if (cur_listener.last_image_ && cur_listener.last_info_)
//            {
//                ROS_INFO("LOOP - NEW DATA");
//            }
            ros::spinOnce();
        }
    }

    void publish_resized(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& info)
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
        if (resized_width_ < image.width && resized_height_ < image.height)
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
            converted = cv_bridge::CvImage(image.header, "mono8", resized);
        }
        else
        {
            converted = cv_bridge::CvImage(image.header, image.encoding, resized);
        }
        converted.toImageMsg(resized_image);
        sensor_msgs::CameraInfo resized_info(info);
        // Republish
        camera_pub_.publish(resized_image, resized_info);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_manager");
    ROS_INFO("Starting camera manager...");
    ros::NodeHandle nh;
    CameraManager manager(nh);
    ROS_INFO("...startup complete");
    manager.loop();
    return 0;
}
