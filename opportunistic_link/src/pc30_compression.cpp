#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <teleop_msgs/CompressedPointCloud2.h>
#include <zlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <opportunistic_link/pc30_compression.h>

#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

using namespace pc30_compression;

void PC30Compressor::reset_encoder()
{
    pframe_counter_ = 0;
    state_packed_.clear();
}

void PC30Compressor::reset_decoder()
{
    pframe_counter_ = 0;
    state_packed_.clear();
}

PC30Compressor::FRAME_TYPES PC30Compressor::header_to_frame_type(uint32_t header_block)
{
    if (header_block == PC30Compressor::IFRAME_ID)
    {
        return PC30Compressor::IFRAME;
    }
    else if (header_block == PC30Compressor::PFRAME_ID)
    {
        return PC30Compressor::PFRAME;
    }
    else
    {
        return PC30Compressor::UNKNOWN;
    }
}

uint32_t PC30Compressor::frame_type_to_header(PC30Compressor::FRAME_TYPES frame_type)
{
    if (frame_type == PC30Compressor::IFRAME)
    {
        return PC30Compressor::IFRAME_ID;
    }
    else if (frame_type == PC30Compressor::PFRAME)
    {
        return PC30Compressor::PFRAME_ID;
    }
    else
    {
        return PC30Compressor::UNKNOWN_ID;
    }
}

std::vector<uint8_t> PC30Compressor::decompress_bytes(std::vector<uint8_t>& compressed)
{
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
    strm.avail_in = compressed.size();
    strm.next_in = reinterpret_cast<uint8_t *>(compressed.data());
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
    std::vector<uint8_t> decompressed(buffer);
    return decompressed;
}

std::vector<uint8_t> PC30Compressor::compress_bytes(std::vector<uint8_t>& uncompressed)
{
    z_stream strm;
    std::vector<uint8_t> buffer;
    const size_t BUFSIZE = 1024 * 1024;
    uint8_t temp_buffer[BUFSIZE];
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = uncompressed.size();
    strm.next_in = reinterpret_cast<uint8_t *>(uncompressed.data());
    strm.next_out = temp_buffer;
    strm.avail_out = BUFSIZE;
    int ret = deflateInit(&strm, Z_BEST_SPEED);
    if (ret != Z_OK)
    {
        (void)deflateEnd(&strm);
        ROS_ERROR("ZLIB unable to init deflate stream");
        throw std::invalid_argument("ZLIB unable to init deflate stream");
    }
    while (strm.avail_in != 0)
    {
        ret = deflate(&strm, Z_NO_FLUSH);
        if (ret != Z_OK)
        {
            (void)deflateEnd(&strm);
            ROS_ERROR("ZLIB unable to deflate stream");
            throw std::invalid_argument("ZLIB unable to deflate stream");
        }
        if (strm.avail_out == 0)
        {
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
        }
    }
    int deflate_ret = Z_OK;
    while (deflate_ret == Z_OK)
    {
        if (strm.avail_out == 0)
        {
            buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
        }
        deflate_ret = deflate(&strm, Z_FINISH);
    }
    if (deflate_ret != Z_STREAM_END)
    {
        (void)deflateEnd(&strm);
        ROS_ERROR("ZLIB unable to deflate stream");
        throw std::invalid_argument("ZLIB unable to deflate stream");
    }
    buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
    (void)deflateEnd(&strm);
    std::vector<uint8_t> compressed(buffer);
    return compressed;
}

sensor_msgs::PointCloud2 PC30Compressor::decode_pointcloud2(teleop_msgs::CompressedPointCloud2& compressed)
{
    std::vector<uint8_t> decompressed_encoded_data = decompress_bytes(compressed.compressed_data);
    // Check if the encoded data is too short to contain a header
    if (decompressed_encoded_data.size() < 4)
    {
        ROS_ERROR("Compressed data does not contain a valid PC30 header");
        throw std::invalid_argument("Compressed data does not contain a valid PC30 header");
    }
    // Check if the encoded data is just a header for an empty cloud
    else if (decompressed_encoded_data.size() == 4)
    {
        ROS_WARN("Compressed data contains an empty pointcloud");
        sensor_msgs::PointCloud2 decoded;
        decoded.header.stamp = compressed.header.stamp;
        decoded.header.frame_id = compressed.header.frame_id;
        decoded.is_dense = compressed.is_dense;
        decoded.is_bigendian = compressed.is_bigendian;
        decoded.fields = compressed.fields;
        decoded.height = compressed.height;
        decoded.width = compressed.width;
        decoded.point_step = compressed.point_step;
        decoded.row_step = compressed.row_step;
        return decoded;
    }
    // Check if the encoded data is a valid length
    else if ((decompressed_encoded_data.size() % 4) != 0)
    {
        ROS_ERROR("Compressed data does not contain valid PC30 encoded data");
        throw std::invalid_argument("Compressed data does not contain valid PC30 encoded data");
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
            // Extract uint32_t data
            std::vector<uint32_t> encoded_cloud_data;
            encoded_cloud_data.resize((decompressed_encoded_data.size() / 4) - 1);
            for (size_t eidx = 0, didx = 4; eidx < encoded_cloud_data.size(); eidx++, didx+=4)
            {
                uint32_t extracted_point = 0;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 3];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 2];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 1];
                extracted_point = extracted_point << 8;
                extracted_point = extracted_point | decompressed_encoded_data[didx + 0];
                encoded_cloud_data[eidx] = extracted_point;
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
            decoded.is_dense = compressed.is_dense;
            decoded.is_bigendian = compressed.is_bigendian;
            decoded.height = compressed.height;
            decoded.width = compressed.width;
            decoded.point_step = compressed.point_step;
            decoded.row_step = compressed.row_step;
            return decoded;
        }
        // Check the header to tell if this is a partial update "P-Frame"
        else if (frame_type == PFRAME)
        {
            // P-Frames have a bigger header - first, check size
            if (decompressed_encoded_data.size() < 8)
            {
                ROS_ERROR("Compressed data does not contain a valid PC30 PFRAME header");
                throw std::invalid_argument("Compressed data does not contain a valid PC30 PFRAME header");
            }
            // P-Frames that only have a header mean that the pointcloud is empty
            else if (decompressed_encoded_data.size() == 8)
            {
                ROS_WARN("PC30 PFRAME contains an empty pointcloud");
                sensor_msgs::PointCloud2 decoded;
                decoded.header.stamp = compressed.header.stamp;
                decoded.header.frame_id = compressed.header.frame_id;
                decoded.is_dense = compressed.is_dense;
                decoded.is_bigendian = compressed.is_bigendian;
                decoded.fields = compressed.fields;
                decoded.height = compressed.height;
                decoded.width = compressed.width;
                decoded.point_step = compressed.point_step;
                decoded.row_step = compressed.row_step;
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
                // Build the destination
                std::vector<uint32_t> recovered_cloud_data;
                recovered_cloud_data.resize(data_length);
                // Initialize the counters
                // Index into recovered_cloud_data
                size_t ridx = 0;
                // Index into decompressed_encoded_data
                size_t didx = 8;
                // Index into state_packed_
                size_t eidx = 0;
                while (ridx < recovered_cloud_data.size())
                {
                    uint32_t extracted_point = 0;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 3];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 2];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 1];
                    extracted_point = extracted_point << 8;
                    extracted_point = extracted_point | decompressed_encoded_data[didx + 0];
                    uint32_t control_flag = extracted_point & 0xc0000000;
                    // If the control flag is empty, this is point data
                    if (control_flag == 0)
                    {
                        recovered_cloud_data[ridx] = extracted_point;
                        ridx++;
                        didx+=4;
                        eidx++;
                    }
                    // If the control flag is non-zero, this is a run-length-encoded 'skip'
                    else
                    {
                        uint32_t skip_distance = extracted_point & 0x3fffffff;
                        uint32_t start_skip = eidx;
                        uint32_t end_skip = eidx + skip_distance;
                        for (size_t sidx = start_skip; sidx < end_skip; sidx++)
                        {
                            recovered_cloud_data[ridx] = state_packed_[eidx];
                            eidx++;
                            ridx++;
                        }
                        didx+=4;
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
                decoded.is_dense = compressed.is_dense;
                decoded.is_bigendian = compressed.is_bigendian;
                decoded.height = compressed.height;
                decoded.width = compressed.width;
                decoded.point_step = compressed.point_step;
                decoded.row_step = compressed.row_step;
                return decoded;
            }
        }
        // If the header isn't recognized
        else
        {
            ROS_ERROR("Compressed data does not contain a valid PC30 frame type");
            ROS_ERROR("Invalid frame type: %x", header);
            throw std::invalid_argument("Compressed data does not contain a valid PC30 frame type");
        }
    }
}

pcl::PointCloud<pcl::PointXYZ> PC30Compressor::get_current_pointcloud()
{
    pcl::PointCloud<pcl::PointXYZ> current_pointcloud;
    for (size_t tidx = 0; tidx < state_packed_.size(); tidx++)
    {
        uint32_t current_point = state_packed_[tidx];
        // Get X,Y,Z integer values
        uint32_t z_cm = current_point & 0x000003ff;
        current_point = current_point >> 10;
        uint32_t y_cm = current_point & 0x000003ff;
        current_point = current_point >> 10;
        uint32_t x_cm = current_point & 0x000003ff;
        // Convert to meters
        float x_m = (float)x_cm / 100.0;
        float y_m = (float)y_cm / 100.0;
        float z_m = (float)z_cm / 100.0;
        // Transform back to the original frame
        float x = x_m - 5.0;
        float y = y_m - 5.0;
        float z = z_m - 5.0;
        // Turn into a PCL point
        pcl::PointXYZ new_point(x, y, z);
        current_pointcloud.push_back(new_point);
    }
    return current_pointcloud;
}

teleop_msgs::CompressedPointCloud2 PC30Compressor::encode_pointcloud2(sensor_msgs::PointCloud2& cloud)
{
    // First, we convert the entire pointcloud to PC30 encoding
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    std::vector<uint32_t> tenbit_positions;
    for (size_t idx = 0; idx < pcl_cloud.size(); idx++)
    {
        pcl::PointXYZ& current_point = pcl_cloud.at(idx);
        float x = current_point.x;
        float y = current_point.y;
        float z = current_point.z;
        if (fabs(x) >= 5.0 || fabs(y) >= 5.0 || fabs(z) >= 5.0)
        {
            ROS_DEBUG("Ignoring point outside bounding box");
        }
        else if (isnan(x) || isnan(y) || isnan(z))
        {
            ROS_DEBUG("Ignoring point with NAN coordinates");
        }
        else
        {
            // Transform to new frame
            float new_x = x + 5.0;
            float new_y = y + 5.0;
            float new_z = z + 5.0;
            // Convert values to centimeters
            float new_x_cm = new_x * 100.0;
            float new_y_cm = new_y * 100.0;
            float new_z_cm = new_z * 100.0;
            // Round to integer values
            uint32_t x_cm = snap(new_x_cm);
            uint32_t y_cm = snap(new_y_cm);
            uint32_t z_cm = snap(new_z_cm);
            // Drop to 10-bit precision
            x_cm = x_cm & 0x000003ff;
            y_cm = y_cm & 0x000003ff;
            z_cm = z_cm & 0x000003ff;
            // Pack into a uint32_t
            uint32_t tenbit_point = 0;
            tenbit_point = tenbit_point | x_cm;
            tenbit_point = tenbit_point << 10;
            tenbit_point = tenbit_point | y_cm;
            tenbit_point = tenbit_point << 10;
            tenbit_point = tenbit_point | z_cm;
            tenbit_positions.push_back(tenbit_point);
        }
    }
    // Then, depending on pframe_counter_, we return the complete I-Frame
    if ((pframe_counter_ % iframe_rate_) == 0)
    {
        // Convert to binary data
        std::vector<uint8_t> raw_data;
        raw_data.resize(4 * tenbit_positions.size() + 4);
        // Set the first 4 bytes to store frame type
        uint32_t frame_type_header = frame_type_to_header(IFRAME);
        raw_data[0] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[1] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[2] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[3] = frame_type_header & 0x000000ff;
        for (size_t tidx = 0, didx = 4; tidx < tenbit_positions.size(); tidx++, didx+=4)
        {
            uint32_t current_point = tenbit_positions[tidx];
            raw_data[didx + 0] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 1] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 2] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 3] = current_point & 0x000000ff;
        }
        // Load into CompressedPointCloud2
        teleop_msgs::CompressedPointCloud2 compressed_cloud;
        compressed_cloud.header.stamp = cloud.header.stamp;
        compressed_cloud.header.frame_id = cloud.header.frame_id;
        compressed_cloud.is_dense = cloud.is_dense;
        compressed_cloud.is_bigendian = cloud.is_bigendian;
        compressed_cloud.fields = cloud.fields;
        compressed_cloud.height = cloud.height;
        compressed_cloud.width = cloud.width;
        compressed_cloud.point_step = cloud.point_step;
        compressed_cloud.row_step = cloud.row_step;
        compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::PC30;
        compressed_cloud.compressed_data = compress_bytes(raw_data);
        // Increment PFRAME counter
        pframe_counter_++;
        // Store the current cloud into the encoder state
        state_packed_.swap(tenbit_positions);
        return compressed_cloud;
    }
    // If we want to return a P-Frame but we have no data to use, send an IFRAME
    else if (state_packed_.size() == 0)
    {
        ROS_WARN("Attempted to send PFRAME, but no state available. IFRAME sent instead");
        // Convert to binary data
        std::vector<uint8_t> raw_data;
        raw_data.resize(4 * tenbit_positions.size() + 4);
        // Set the first 4 bytes to store frame type
        uint32_t frame_type_header = frame_type_to_header(IFRAME);
        raw_data[0] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[1] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[2] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[3] = frame_type_header & 0x000000ff;
        for (size_t tidx = 0, didx = 4; tidx < tenbit_positions.size(); tidx++, didx+=4)
        {
            uint32_t current_point = tenbit_positions[tidx];
            raw_data[didx + 0] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 1] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 2] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 3] = current_point & 0x000000ff;
        }
        // Load into CompressedPointCloud2
        teleop_msgs::CompressedPointCloud2 compressed_cloud;
        compressed_cloud.header.stamp = cloud.header.stamp;
        compressed_cloud.header.frame_id = cloud.header.frame_id;
        compressed_cloud.is_dense = cloud.is_dense;
        compressed_cloud.is_bigendian = cloud.is_bigendian;
        compressed_cloud.fields = cloud.fields;
        compressed_cloud.height = cloud.height;
        compressed_cloud.width = cloud.width;
        compressed_cloud.point_step = cloud.point_step;
        compressed_cloud.row_step = cloud.row_step;
        compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::PC30;
        compressed_cloud.compressed_data = compress_bytes(raw_data);
        // Increment PFRAME counter
        pframe_counter_++;
        // Store the current cloud into the encoder state
        state_packed_.swap(tenbit_positions);
        return compressed_cloud;
    }
    // If not, we return a partial P-Frame
    else
    {
        // Compute the delta between the current state and the stored encoder state
        std::vector<uint32_t> delta_data;
        size_t current_data_length = state_packed_.size();
        size_t tenbitidx = 0;
        uint32_t skip_counter = 0;
        while (tenbitidx < tenbit_positions.size())
        {
            if (tenbitidx < current_data_length)
            {
                // If current and stored data are the same, we store the skips
                if (tenbit_positions[tenbitidx] == state_packed_[tenbitidx])
                {
                    skip_counter++;
                }
                else
                {
                    if (skip_counter != 0)
                    {
                        uint32_t skip_block = skip_counter | 0xc0000000;
                        delta_data.push_back(skip_block);
                    }
                    delta_data.push_back(tenbit_positions[tenbitidx]);
                    skip_counter = 0;
                }
            }
            // If we have more points than we have stored, there's no reason to check
            else
            {
                delta_data.push_back(tenbit_positions[tenbitidx]);
            }
            tenbitidx++;
        }
        // Convert to binary data
        std::vector<uint8_t> raw_data;
        raw_data.resize(4 * delta_data.size() + 8);
        // Set the first 4 bytes to store frame type
        uint32_t frame_type_header = frame_type_to_header(IFRAME);
        raw_data[0] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[1] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[2] = frame_type_header & 0x000000ff;
        frame_type_header = frame_type_header >> 8;
        raw_data[3] = frame_type_header & 0x000000ff;
        // Set the next 4 bytes to store the decoded length
        uint32_t decoded_data_length = (uint32_t)tenbit_positions.size();
        raw_data[4] = decoded_data_length & 0x000000ff;
        decoded_data_length = decoded_data_length >> 8;
        raw_data[5] = decoded_data_length & 0x000000ff;
        decoded_data_length = decoded_data_length >> 8;
        raw_data[6] = decoded_data_length & 0x000000ff;
        decoded_data_length = decoded_data_length >> 8;
        raw_data[7] = decoded_data_length & 0x000000ff;
        for (size_t tidx = 0, didx = 8; tidx < delta_data.size(); tidx++, didx+=4)
        {
            uint32_t current_point = delta_data[tidx];
            raw_data[didx + 0] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 1] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 2] = current_point & 0x000000ff;
            current_point = current_point >> 8;
            raw_data[didx + 3] = current_point & 0x000000ff;
        }
        // Load into CompressedPointCloud2
        teleop_msgs::CompressedPointCloud2 compressed_cloud;
        compressed_cloud.header.stamp = cloud.header.stamp;
        compressed_cloud.header.frame_id = cloud.header.frame_id;
        compressed_cloud.is_dense = cloud.is_dense;
        compressed_cloud.is_bigendian = cloud.is_bigendian;
        compressed_cloud.fields = cloud.fields;
        compressed_cloud.height = cloud.height;
        compressed_cloud.width = cloud.width;
        compressed_cloud.point_step = cloud.point_step;
        compressed_cloud.row_step = cloud.row_step;
        compressed_cloud.compression_type = teleop_msgs::CompressedPointCloud2::PC30;
        compressed_cloud.compressed_data = compress_bytes(raw_data);
        // Increment PFRAME counter
        pframe_counter_++;
        // Store the current cloud into the encoder state
        state_packed_.swap(tenbit_positions);
        return compressed_cloud;
    }
}
