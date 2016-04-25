#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opportunistic_link/image_compression.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

using namespace image_compression;

void ImageHandler::reset_encoder()
{
    ;
}

void ImageHandler::reset_decoder()
{
    ;
}

sensor_msgs::Image ImageHandler::decompress_image(const sensor_msgs::CompressedImage& compressed, const std::string& encoding)
{
    sensor_msgs::Image decompressed;
    cv::Mat decoded = cv::imdecode(compressed.data, CV_LOAD_IMAGE_ANYCOLOR);
    cv_bridge::CvImage converted;
    converted = cv_bridge::CvImage(compressed.header, encoding, decoded);
    converted.toImageMsg(decompressed);
    return decompressed;
}

sensor_msgs::CompressedImage ImageHandler::compress_image(const sensor_msgs::Image& image, const int quality)
{
    sensor_msgs::CompressedImage compressed;
    compressed.header.frame_id = image.header.frame_id;
    compressed.header.stamp = image.header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image);
    cv::Mat cv_image = cv_ptr->image;
    std::vector<int> encoding_params;
    encoding_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    encoding_params.push_back(quality);
    bool ret = cv::imencode(".jpg", cv_image, compressed.data, encoding_params);
    if (ret)
    {
        ROS_DEBUG("OpenCV encoding success");
    }
    else
    {
        ROS_ERROR("OpenCV encoding failure");
    }
    return compressed;
}
