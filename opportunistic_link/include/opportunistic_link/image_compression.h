#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv/cv.h>

#ifndef IMAGE_COMPRESSION_H
#define IMAGE_COMPRESSION_H

namespace image_compression
{
    class ImageHandler
    {
    protected:

        ;

    public:

        ImageHandler() {}

        void reset_encoder();

        void reset_decoder();

        sensor_msgs::Image decompress_image(const sensor_msgs::CompressedImage& compressed, const std::string& encoding);

        sensor_msgs::CompressedImage compress_image(const sensor_msgs::Image& image, const int quality);

    };
}

#endif // IMAGE_COMPRESSION_H
