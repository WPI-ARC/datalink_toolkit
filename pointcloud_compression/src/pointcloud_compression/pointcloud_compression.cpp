#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <zlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pointcloud_compression/zlib_helpers.hpp>
#include <pointcloud_compression/pointcloud_compression.h>
#include <pointcloud_compression/pc30_compression.h>
#include <pointcloud_compression/pc60_compression.h>

using namespace pointcloud_compression;

void PointCloudHandler::reset_encoder()
{
    pcl_encoder_.initialization();
    pc30_encoder_.reset_encoder();
    pc60_encoder_.reset_encoder();
}

void PointCloudHandler::reset_decoder()
{
    pcl_decoder_.initialization();
    pc30_decoder_.reset_decoder();
    pc60_decoder_.reset_decoder();
}

sensor_msgs::PointCloud2 PointCloudHandler::decompress_pointcloud2(const datalink_msgs::CompressedPointCloud2& compressed)
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
    if (compressed.compression_type == datalink_msgs::CompressedPointCloud2::ZLIB)
    {
        // Decompress the pointcloud data using ZLIB's DEFLATE compression
        cloud.data = ZlibHelpers::DecompressBytes(compressed.compressed_data);
    }
    else if (compressed.compression_type == datalink_msgs::CompressedPointCloud2::PCL)
    {
        if (compressed.compressed_data.size() == 0)
        {
            ROS_WARN("Decompressor received an empty pointcloud");
        }
        else
        {
            uncompressed_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
            std::string compressed_data;
            compressed_data.insert(compressed_data.end(), compressed.compressed_data.begin(), compressed.compressed_data.end());
            std::stringstream compressed_data_stream;
            compressed_data_stream.str(compressed_data);
            pcl_decoder_.decodePointCloud(compressed_data_stream, uncompressed_cloud_ptr_);
            sensor_msgs::PointCloud2 new_cloud;
            pcl::toROSMsg(*uncompressed_cloud_ptr_, new_cloud);
            cloud.data = new_cloud.data;
            cloud.is_dense = new_cloud.is_dense;
            cloud.is_bigendian = new_cloud.is_bigendian;
            cloud.fields = new_cloud.fields;
            cloud.height = new_cloud.height;
            cloud.width = new_cloud.width;
            cloud.point_step = new_cloud.point_step;
            cloud.row_step = new_cloud.row_step;
        }
    }
    else if (compressed.compression_type == datalink_msgs::CompressedPointCloud2::PC30)
    {
        cloud = pc30_decoder_.decode_pointcloud2(compressed);
    }
    else if (compressed.compression_type == datalink_msgs::CompressedPointCloud2::PC60)
    {
        cloud = pc60_decoder_.decode_pointcloud2(compressed);
    }
    else if (compressed.compression_type == datalink_msgs::CompressedPointCloud2::NONE)
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

datalink_msgs::CompressedPointCloud2 PointCloudHandler::compress_pointcloud2(const sensor_msgs::PointCloud2& cloud, const uint8_t compression_type, const float filter_size)
{
    sensor_msgs::PointCloud2 intermediate;
    if (filter_size > 0.0)
    {
        // Convert the ROS cloud to PCL for voxel filtering
        pcl::PCLPointCloud2 pcl_cloud;
        pcl::PCLPointCloud2 pcl_intermediate;
        pcl_conversions::toPCL(cloud, pcl_cloud);
        // Voxel filter the cloud
        pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(&pcl_cloud, dealocate_pcl_pointcloud2_fn);
        voxel_filter_.setInputCloud(pcl_cloud_ptr);
        voxel_filter_.setLeafSize(filter_size, filter_size, filter_size);
        voxel_filter_.filter(pcl_intermediate);
        // Convert the PCL cloud back to ROS for compression
        pcl_conversions::moveFromPCL(pcl_cloud, intermediate);
        intermediate.header = cloud.header;
    }
    else
    {
        intermediate = cloud;
    }
    datalink_msgs::CompressedPointCloud2 compressed = compress_pointcloud2(intermediate, compression_type);
    return compressed;
}

datalink_msgs::CompressedPointCloud2 PointCloudHandler::compress_pointcloud2(const sensor_msgs::PointCloud2& cloud, const uint8_t compression_type)
{
    datalink_msgs::CompressedPointCloud2 compressed_cloud;
    compressed_cloud.header.stamp = cloud.header.stamp;
    compressed_cloud.header.frame_id = cloud.header.frame_id;
    compressed_cloud.is_dense = cloud.is_dense;
    compressed_cloud.is_bigendian = cloud.is_bigendian;
    compressed_cloud.fields = cloud.fields;
    compressed_cloud.height = cloud.height;
    compressed_cloud.width = cloud.width;
    compressed_cloud.point_step = cloud.point_step;
    compressed_cloud.row_step = cloud.row_step;
    if (compression_type == datalink_msgs::CompressedPointCloud2::ZLIB)
    {
        // Compress the pointcloud data using ZLIB's DEFLATE compression
        compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::ZLIB;
        compressed_cloud.compressed_data = ZlibHelpers::CompressBytes(cloud.data);
    }
    else if (compression_type == datalink_msgs::CompressedPointCloud2::PCL)
    {
        compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::PCL;
        pcl::PointCloud<pcl::PointXYZRGB> converted_cloud;
        pcl::fromROSMsg(cloud, converted_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudptr(&converted_cloud, dealocate_pcl_pointcloud_fn);
        std::stringstream compressed_data_stream;
        pcl_encoder_.encodePointCloud(cloudptr, compressed_data_stream);
        std::string compressed_data = compressed_data_stream.str();
        std::vector<uint8_t> buffer(compressed_data.begin(), compressed_data.end());
        compressed_cloud.compressed_data = buffer;
    }
    else if (compression_type == datalink_msgs::CompressedPointCloud2::PC30)
    {
        compressed_cloud = pc30_encoder_.encode_pointcloud2(cloud);
    }
    else if (compression_type == datalink_msgs::CompressedPointCloud2::PC60)
    {
        compressed_cloud = pc60_encoder_.encode_pointcloud2(cloud);
    }
    else
    {
        compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::NONE;
        compressed_cloud.compressed_data = cloud.data;
    }
    return compressed_cloud;
}
