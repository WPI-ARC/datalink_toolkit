#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <datalink_msgs/CompressedPointCloud2.h>
#include <zlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_compression/zlib_helpers.hpp>
#include <pointcloud_compression/pc60_compression.h>

using namespace pc60_compression;

void PC60Compressor::reset_encoder()
{
    pframe_counter_ = 0;
    state_packed_.clear();
}

void PC60Compressor::reset_decoder()
{
    pframe_counter_ = 0;
    state_packed_.clear();
}

PC60Compressor::FRAME_TYPES PC60Compressor::header_to_frame_type(const uint32_t header_block) const
{
    if (header_block == PC60Compressor::IFRAME_ID)
    {
        return PC60Compressor::IFRAME;
    }
    else if (header_block == PC60Compressor::PFRAME_ID)
    {
        return PC60Compressor::PFRAME;
    }
    else
    {
        return PC60Compressor::UNKNOWN;
    }
}

uint32_t PC60Compressor::frame_type_to_header(const PC60Compressor::FRAME_TYPES frame_type) const
{
    if (frame_type == PC60Compressor::IFRAME)
    {
        return PC60Compressor::IFRAME_ID;
    }
    else if (frame_type == PC60Compressor::PFRAME)
    {
        return PC60Compressor::PFRAME_ID;
    }
    else
    {
        return PC60Compressor::UNKNOWN_ID;
    }
}

sensor_msgs::PointCloud2 PC60Compressor::decode_pointcloud2(const datalink_msgs::CompressedPointCloud2& compressed)
{
    std::vector<uint8_t> decompressed_encoded_data = ZlibHelpers::DecompressBytes(compressed.compressed_data);
    // Check if the encoded data is too short to contain a header
    if (decompressed_encoded_data.size() < 4)
    {
        ROS_ERROR("Compressed data does not contain a valid PC60 header");
        throw std::invalid_argument("Compressed data does not contain a valid PC60 header");
    }
    // Check if the encoded data is just a header for an empty cloud
    else if (decompressed_encoded_data.size() == 4)
    {
        ROS_WARN("Compressed data contains an empty pointcloud");
        pcl::PointCloud<pcl::PointXYZ> empty_cloud = generate_empty_pointcloud();
        // Convert to ROS pointcloud
        sensor_msgs::PointCloud2 decoded;
        pcl::toROSMsg(empty_cloud, decoded);
        // Fill in the header information
        decoded.header.stamp = compressed.header.stamp;
        decoded.header.frame_id = compressed.header.frame_id;
        // Flush the stored state, since we aren't tracking any points
        state_packed_.clear();
        stored_state_.clear();
        return decoded;
    }
    // Check if the encoded data is a valid length
    else if ((decompressed_encoded_data.size() % 4) != 0)
    {
        ROS_ERROR("Compressed data does not contain valid PC60 encoded data");
        throw std::invalid_argument("Compressed data does not contain valid PC60 encoded data");
    }
    // Encoded data contains both a header and real data
    else
    {
        uint32_t header = 0;
        header = header | decompressed_encoded_data[3];
        header = header << 8;
        header = header | decompressed_encoded_data[2];
        header = header << 8;
        header = header | decompressed_encoded_data[1];
        header = header << 8;
        header = header | decompressed_encoded_data[0];
        FRAME_TYPES frame_type = header_to_frame_type(header);
        // Check the header to tell if this is a full-update "I-Frame"
        if (frame_type == IFRAME)
        {
            ROS_INFO("Start I-FRAME decode");
            stored_state_.clear();
            // Extract uint32_t data
            std::vector<uint64_t> encoded_cloud_data;
            encoded_cloud_data.resize((decompressed_encoded_data.size() - 4) / 8);
            for (size_t eidx = 0, didx = 4; eidx < encoded_cloud_data.size(); eidx++, didx+=8)
            {
                uint64_t extracted_point = 0;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 7];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 6];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 5];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 4];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 3];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 2];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 1];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 0];
                encoded_cloud_data[eidx] = extracted_point;
                stored_state_[extracted_point] = 1;
            }
            // Swap the new full frame into the stored cloud data
            state_packed_.swap(encoded_cloud_data);
            // Convert the current decoder state to a PCL pointcloud
            pcl::PointCloud<pcl::PointXYZ> current_cloud = get_current_pointcloud();
            // Convert to ROS pointcloud
            sensor_msgs::PointCloud2 decoded;
            pcl::toROSMsg(current_cloud, decoded);
            // Fill in the header information
            decoded.header.stamp = compressed.header.stamp;
            decoded.header.frame_id = compressed.header.frame_id;
            ROS_INFO("End I-FRAME decode");
            return decoded;
        }
        // Check the header to tell if this is a partial update "P-Frame"
        else if (frame_type == PFRAME)
        {
            ROS_INFO("Start P-FRAME decode");
            // P-Frames have a bigger header - first, check size
            if (decompressed_encoded_data.size() < 8)
            {
                ROS_ERROR("Compressed data does not contain a valid PC60 PFRAME header");
                throw std::invalid_argument("Compressed data does not contain a valid PC60 PFRAME header");
            }
            // P-Frames that only have a header mean that there is no delta from the previous frame
            else if (decompressed_encoded_data.size() == 8)
            {
                ROS_INFO("Decoding a PFRAME with zero delta");
                // Convert the current decoder state to a PCL pointcloud
                pcl::PointCloud<pcl::PointXYZ> current_cloud = get_current_pointcloud();
                // Convert to ROS pointcloud
                sensor_msgs::PointCloud2 decoded;
                pcl::toROSMsg(current_cloud, decoded);
                // Fill in the header information
                decoded.header.stamp = compressed.header.stamp;
                decoded.header.frame_id = compressed.header.frame_id;
                ROS_INFO("End P-FRAME decode");
                return decoded;
            }
            else
            {
                // Get the complete data length, which is stored in the second 32 bits
                // of a P-Frame data block
                uint32_t data_length = 0;
                data_length = data_length | decompressed_encoded_data[7];
                data_length = data_length << 8;
                data_length = data_length | decompressed_encoded_data[6];
                data_length = data_length << 8;
                data_length = data_length | decompressed_encoded_data[5];
                data_length = data_length << 8;
                data_length = data_length | decompressed_encoded_data[4];
                // Add and remove points from the stored state
                for (size_t didx = 8; didx < decompressed_encoded_data.size(); didx+=8)
                {
                    uint64_t extracted_point = 0;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 7];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 6];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 5];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 4];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 3];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 2];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 1];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 0];
                    uint64_t control_flag = extracted_point & 0xf000000000000000;
                    // If control flag is 0xa..., we are adding a new point to the stored state
                    if (control_flag == 0xa000000000000000)
                    {
                        uint64_t real_point = extracted_point & 0x0fffffffffffffff;
                        stored_state_[real_point] = 1;
                    }
                    // If control flag is 0xd..., we are removing the point from the stored state
                    else if (control_flag == 0xd000000000000000)
                    {
                        uint64_t real_point = extracted_point & 0x0fffffffffffffff;
                        stored_state_.erase(real_point);
                    }
                    else
                    {
                        ROS_WARN("Invalid control flag on P-FRAME delta-encoded point: 0x%lx ", control_flag);
                    }
                }
                // Build the final destination
                std::vector<uint64_t> recovered_cloud_data;
                std::map<uint64_t, int8_t>::iterator itr;
                // Loop through the stored state to generate the vector of points
                for (itr = stored_state_.begin(); itr != stored_state_.end(); ++itr)
                {
                    uint64_t point = itr->first;
                    int8_t ctrl = itr->second;
                    // Make sure this is a valid entry
                    if (ctrl > 0)
                    {
                        recovered_cloud_data.push_back(point);
                    }
                }
                // Now that the PFRAME has been rebuilt with the stored state
                state_packed_.swap(recovered_cloud_data);
                // Convert the current decoder state to a PCL pointcloud
                pcl::PointCloud<pcl::PointXYZ> current_cloud = get_current_pointcloud();
                // Convert to ROS pointcloud
                sensor_msgs::PointCloud2 decoded;
                pcl::toROSMsg(current_cloud, decoded);
                // Fill in the header information
                decoded.header.stamp = compressed.header.stamp;
                decoded.header.frame_id = compressed.header.frame_id;
                ROS_INFO("End P-FRAME decode");
                return decoded;
            }
        }
        // If the header isn't recognized
        else
        {
            ROS_ERROR("Compressed data does not contain a valid PC60 frame type");
            ROS_ERROR("Invalid frame type: %x", header);
            throw std::invalid_argument("Compressed data does not contain a valid PC60 frame type");
        }
    }
}

pcl::PointCloud<pcl::PointXYZ> PC60Compressor::generate_empty_pointcloud() const
{
    pcl::PointCloud<pcl::PointXYZ> empty_pointcloud;
    pcl::PointXYZ dummy_point(0.0, 0.0, 0.0);
    empty_pointcloud.push_back(dummy_point);
    return empty_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> PC60Compressor::get_current_pointcloud() const
{
    pcl::PointCloud<pcl::PointXYZ> current_pointcloud;
    for (size_t tidx = 0; tidx < state_packed_.size(); tidx++)
    {
        uint64_t current_point = state_packed_[tidx];
        // Get X,Y,Z integer values
        uint32_t z_cm = current_point & 0x00000000000fffff;
        current_point = current_point >> 20;
        uint32_t y_cm = current_point & 0x00000000000fffff;
        current_point = current_point >> 20;
        uint32_t x_cm = current_point & 0x00000000000fffff;
        // Convert to meters
        float x_m = (float)x_cm / precision_;
        float y_m = (float)y_cm / precision_;
        float z_m = (float)z_cm / precision_;
        // Transform back to the original frame
        float x = x_m - 500.0f;
        float y = y_m - 500.0f;
        float z = z_m - 500.0f;
        // Turn into a PCL point
        pcl::PointXYZ new_point(x, y, z);
        current_pointcloud.push_back(new_point);
    }
    return current_pointcloud;
}

datalink_msgs::CompressedPointCloud2 PC60Compressor::encode_pointcloud2(const sensor_msgs::PointCloud2& cloud)
{
    // First, we convert the entire pointcloud to PC60 encoding
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    std::vector<uint64_t> twentybit_positions;
    std::map<uint64_t, int8_t> new_state;
    for (size_t idx = 0; idx < pcl_cloud.size(); idx++)
    {
        pcl::PointXYZ& current_point = pcl_cloud.at(idx);
        float x = current_point.x;
        float y = current_point.y;
        float z = current_point.z;
        if (fabs(x) >= 500.0 || fabs(y) >= 500.0 || fabs(z) >= 500.0)
        {
            ROS_DEBUG("Ignoring point outside bounding box");
        }
        else if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_DEBUG("Ignoring point with NAN coordinates");
        }
        else
        {
            // Transform to new frame
            float new_x = x + 500.0f;
            float new_y = y + 500.0f;
            float new_z = z + 500.0f;
            // Convert values to millimeters
            float new_x_mm = new_x * precision_;
            float new_y_mm = new_y * precision_;
            float new_z_mm = new_z * precision_;
            // Round to integer values
            uint32_t x_mm = (uint32_t)round(new_x_mm);
            uint32_t y_mm = (uint32_t)round(new_y_mm);
            uint32_t z_mm = (uint32_t)round(new_z_mm);
            // Drop to 20-bit precision
            x_mm = x_mm & 0x000fffff;
            y_mm = y_mm & 0x000fffff;
            z_mm = z_mm & 0x000fffff;
            // Pack into a uint32_t
            uint64_t twentybit_point = 0;
            twentybit_point = twentybit_point | x_mm;
            twentybit_point = twentybit_point << 20;
            twentybit_point = twentybit_point | y_mm;
            twentybit_point = twentybit_point << 20;
            twentybit_point = twentybit_point | z_mm;
            twentybit_positions.push_back(twentybit_point);
            new_state[twentybit_point] = 1;
        }
    }
    // Then, depending on pframe_counter_, we return the complete I-Frame
    std::cout << "PFRAME_COUNTER_: " << pframe_counter_ << " IFRAME_RATE_: " << iframe_rate_ << std::endl;
    if ((pframe_counter_ % iframe_rate_) == 0)
    {
        ROS_INFO("Start IFRAME encode");
        // Convert to binary data
        std::vector<uint8_t> raw_data;
        raw_data.resize(8 * twentybit_positions.size() + 4);
        // Set the first 4 bytes to store frame type
        uint32_t frame_type_header = frame_type_to_header(IFRAME);
        raw_data[0] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[1] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[2] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[3] = frame_type_header & 0x000000ff;
        for (size_t tidx = 0, didx = 4; tidx < twentybit_positions.size(); tidx++, didx+=8)
        {
            uint64_t current_point = twentybit_positions[tidx];
            raw_data[didx + 0] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 1] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 2] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 3] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 4] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 5] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 6] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 7] = current_point & 0x000000ff;
        }
        // Load into CompressedPointCloud2
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
        compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::PC60;
        compressed_cloud.compressed_data = ZlibHelpers::CompressBytes(raw_data);
        // Increment PFRAME counter
        pframe_counter_++;
        // Store the current cloud into the encoder state
        state_packed_.swap(twentybit_positions);
        stored_state_.swap(new_state);
        return compressed_cloud;
    }
    // If we want to return a P-Frame but we have no data to use, send an IFRAME
    else if (state_packed_.size() == 0)
    {
        ROS_WARN("Attempted to send PFRAME, but no state available. IFRAME sent instead");
        // Convert to binary data
        std::vector<uint8_t> raw_data;
        raw_data.resize(8 * twentybit_positions.size() + 4);
        // Set the first 4 bytes to store frame type
        uint32_t frame_type_header = frame_type_to_header(IFRAME);
        raw_data[0] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[1] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[2] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[3] = frame_type_header & 0x000000ff;
        for (size_t tidx = 0, didx = 4; tidx < twentybit_positions.size(); tidx++, didx+=8)
        {
            uint64_t current_point = twentybit_positions[tidx];
            raw_data[didx + 0] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 1] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 2] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 3] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 4] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 5] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 6] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 7] = current_point & 0x000000ff;
        }
        // Load into CompressedPointCloud2
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
        compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::PC60;
        compressed_cloud.compressed_data = ZlibHelpers::CompressBytes(raw_data);
        // Reset PFRAME counter
        pframe_counter_ = 0;
        // Store the current cloud into the encoder state
        state_packed_.swap(twentybit_positions);
        stored_state_.swap(new_state);
        return compressed_cloud;
    }
    // If not, we return a partial P-Frame
    else
    {
        ROS_INFO("Start PFRAME encode");
        // Compute the delta between new map and stored map
        std::map<uint64_t, int8_t> safe_state;
        std::vector<uint64_t> delta_data;
        std::map<uint64_t, int8_t>::iterator itr;
        // Loop through the new state to find the *new points* in the latest pointcloud
        for (itr = new_state.begin(); itr != new_state.end(); ++itr)
        {
            uint64_t point = itr->first;
            int8_t ctrl = itr->second;
            // First, add to the "safe state" that only contains valid points to store
            if (ctrl > 0)
            {
                safe_state[point] = 1;
            }
            // Second, check if the point is already stored
            int8_t stored_ctrl = stored_state_[point];
            // If the stored ctrl is greater than zero, it's being stored
            // If it's zero (the default value), then the point is new
            if (ctrl > 0 && stored_ctrl == 0)
            {
                // Add the new point to the delta as an "ADD" with 0b10 as the header
                uint64_t delta_point = point | 0xa000000000000000;
                delta_data.push_back(delta_point);
            }
        }
        // Loop through the old cloud to find the *old points* NOT in the latest pointcloud
        for (itr = stored_state_.begin(); itr != stored_state_.end(); ++itr)
        {
            uint64_t point = itr->first;
            int8_t ctrl = itr->second;
            // Check if the old point is also in the new pointcloud
            int8_t new_ctrl = new_state[point];
            // If the new ctrl is greater than zero, it's being stored
            // If it's zero (the default value), then the point is no longer there
            if (ctrl > 0 && new_ctrl == 0)
            {
                // Add the new point to the delta as a "REMOVE" with 0b01 as the header
                uint64_t delta_point = point | 0xd000000000000000;
                delta_data.push_back(delta_point);
            }
        }
        // Check if delta-encoded P-Frame would be larger than an I-Frame
        if (delta_data.size() >= twentybit_positions.size())
        {
            ROS_WARN("Attempted to send PFRAME, but it would be larger than an IFRAME. IFRAME sent instead");
            // Convert to binary data
            std::vector<uint8_t> raw_data;
            raw_data.resize(8 * twentybit_positions.size() + 4);
            // Set the first 4 bytes to store frame type
            uint32_t frame_type_header = frame_type_to_header(IFRAME);
            raw_data[0] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[1] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[2] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[3] = frame_type_header & 0x000000ff;
            for (size_t tidx = 0, didx = 4; tidx < twentybit_positions.size(); tidx++, didx+=8)
            {
                uint64_t current_point = twentybit_positions[tidx];
                raw_data[didx + 0] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 1] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 2] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 3] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 4] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 5] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 6] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 7] = current_point & 0x000000ff;
            }
            // Load into CompressedPointCloud2
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
            compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::PC60;
            compressed_cloud.compressed_data = ZlibHelpers::CompressBytes(raw_data);
            // Reset PFRAME counter
            pframe_counter_ = 0;
            // Store the current cloud into the encoder state
            state_packed_.swap(twentybit_positions);
            stored_state_.swap(safe_state);
            return compressed_cloud;
        }
        else
        {
            ROS_INFO("Completing PFRAME encode");
            // P-Frame will be smaller, so we send it
            // Convert to binary data
            std::vector<uint8_t> raw_data;
            raw_data.resize(8 * delta_data.size() + 8);
            // Set the first 4 bytes to store frame type
            uint32_t frame_type_header = frame_type_to_header(PFRAME);
            raw_data[0] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[1] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[2] = frame_type_header & 0x000000ff;
            frame_type_header = frame_type_header >> 8;
            raw_data[3] = frame_type_header & 0x000000ff;
            // Set the next 4 bytes to store the decoded length
            uint32_t decoded_data_length = (uint32_t)twentybit_positions.size();
            raw_data[4] = decoded_data_length & 0x000000ff;
            decoded_data_length = decoded_data_length >> 8;
            raw_data[5] = decoded_data_length & 0x000000ff;
            decoded_data_length = decoded_data_length >> 8;
            raw_data[6] = decoded_data_length & 0x000000ff;
            decoded_data_length = decoded_data_length >> 8;
            raw_data[7] = decoded_data_length & 0x000000ff;
            for (size_t tidx = 0, didx = 8; tidx < delta_data.size(); tidx++, didx+=8)
            {
                uint64_t current_point = delta_data[tidx];
                raw_data[didx + 0] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 1] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 2] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 3] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 4] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 5] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 6] = current_point & 0x000000ff;
                current_point = current_point >> 8;
                raw_data[didx + 7] = current_point & 0x000000ff;
            }
            // Load into CompressedPointCloud2
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
            compressed_cloud.compression_type = datalink_msgs::CompressedPointCloud2::PC60;
            compressed_cloud.compressed_data = ZlibHelpers::CompressBytes(raw_data);
            // Increment PFRAME counter
            pframe_counter_++;
            // Store the current cloud into the encoder state
            state_packed_.swap(twentybit_positions);
            stored_state_.swap(safe_state);
            return compressed_cloud;
        }
    }
}
