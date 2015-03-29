#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <zlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#ifndef PC60_COMPRESSION_H
#define PC60_COMPRESSION_H

namespace pc60_compression
{

    class PC60Compressor
    {
    protected:

        std::vector<u_int64_t> state_packed_;
        std::map<u_int64_t, int8_t> stored_state_;
        u_int32_t iframe_rate_;
        u_int32_t pframe_counter_;
        pcl::PointCloud<pcl::PointXYZ> get_current_pointcloud();
        pcl::PointCloud<pcl::PointXYZ> generate_empty_pointcloud();
        float precision_;

    public:

        enum FRAME_TYPES {IFRAME, PFRAME, UNKNOWN};
        static const u_int32_t IFRAME_ID = 0x00ffffff;
        static const u_int32_t PFRAME_ID = 0x00dfdfdf;
        static const u_int32_t UNKNOWN_ID = 0x00000000;

        PC60Compressor(float precision=1000.0, u_int32_t iframe_rate=5)
        {
            precision_ = precision;
            iframe_rate_ = iframe_rate;
            pframe_counter_ = 0;
        }

        void reset_encoder();

        void reset_decoder();

        FRAME_TYPES header_to_frame_type(u_int32_t header_block);

        u_int32_t frame_type_to_header(FRAME_TYPES frame_type);

        sensor_msgs::PointCloud2 decode_pointcloud2(datalink_msgs::CompressedPointCloud2& compressed);

        datalink_msgs::CompressedPointCloud2 encode_pointcloud2(sensor_msgs::PointCloud2& cloud);

    };
}

#endif // PC60_COMPRESSION_H
