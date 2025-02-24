#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include "ouster_ros/os_ros.h"
#include "point_cloud_processor_factory.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include <boost/foreach.hpp>
#include <fstream>
#include <sstream>
#include <mutex>

using namespace ouster_ros;

class BagConverter
{
public:
  // Added an optional metadata_file parameter (default is empty, meaning use bag metadata)
  BagConverter(const std::string &input_bag, const std::string &output_bag, const std::string &ns = "ouster", const std::string &metadata_file = "")
      : input_bag_(input_bag), output_bag_(output_bag), ns_(ns), metadata_file_(metadata_file) {}

  void process()
  {
    rosbag::Bag in_bag, out_bag;
    in_bag.open(input_bag_, rosbag::bagmode::Read);
    out_bag.open(output_bag_, rosbag::bagmode::Write);

    ouster::sensor::sensor_info info;
    ros::Time metadata_ts;

    // Determine the metadata source: file or bag
    if (metadata_file_.empty())
    {
      // First pass: find and process metadata in the bag
      rosbag::View metadata_view(in_bag, [&](const rosbag::ConnectionInfo *ci)
                                 { return ci->topic == "/" + ns_ + "/metadata"; });

      if (metadata_view.size() == 0)
      {
        ROS_ERROR("No metadata found in bag file");
        return;
      }

      std::cout << "Found metadata in bag file" << std::endl;
      auto metadata_msg = metadata_view.begin()->instantiate<std_msgs::String>();
      metadata_ts = metadata_view.begin()->getTime();
      info = ouster::sensor::parse_metadata(metadata_msg->data);
    }
    else
    {
      // Read metadata from the specified file
      std::ifstream file(metadata_file_);
      if (!file.is_open())
      {
        ROS_ERROR("Could not open metadata file: %s", metadata_file_.c_str());
        return;
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      file.close();
      std::string metadata_string = buffer.str();
      std::cout << "Loaded metadata from file: " << metadata_file_ << std::endl;
      // When metadata is loaded from file, set metadata_ts to zero so that all pointcloud messages are valid.
      metadata_ts = ros::Time(0);
      info = ouster::sensor::parse_metadata(metadata_string);
    }

    std::mutex write_mutex;
    ros::Time lidar_packet_time;

    std::vector<LidarScanProcessor> lidar_scan_processors;
    lidar_scan_processors.push_back(
        PointCloudProcessorFactory::create_point_cloud_processor(
            "original", info, "os_sensor",
            true, true, true, 0, 10000 * 1000, 1,
            [this, &out_bag, &metadata_ts, &write_mutex, &lidar_packet_time](PointCloudProcessor_OutputType msgs)
            {
              // At the moment we only care about the first pointcloud.
              // Any pointcloud that is timestamped before the metadata message is considered invalid.
              // This check is necessary because the initial packets might not form a complete pointcloud
              // so its better to ignore the first one or two pointclouds and only start writing the valid ones
              if (msgs[0]->header.stamp < metadata_ts)
              {
                return;
              }

              std::lock_guard<std::mutex> lock(write_mutex);
              out_bag.write("/" + ns_ + "/points", lidar_packet_time, *msgs[0]);
            }));

    auto lidar_packet_handler = LidarPacketHandler::create(
        info, lidar_scan_processors, "", static_cast<int64_t>(-37.0 * 1e+9), 0);

    auto imu_packet_handler = ImuPacketHandler::create(
        info, "os_imu", "", static_cast<int64_t>(-37.0 * 1e+9));

    // Second pass: process all messages, writing original messages and converting lidar/IMU packets.
    rosbag::View view(in_bag);
    size_t total = view.size();
    size_t count = 0;
    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
      // Update counter
      count++;

      // Write original message to output bag
      {
        std::lock_guard<std::mutex> lock(write_mutex);
        out_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }

      // Process lidar packets
      if (m.getTopic() == "/" + ns_ + "/lidar_packets")
      {
        auto msg = m.instantiate<PacketMsg>();
        if (!msg)
        {
          ROS_ERROR("Received a null lidar packet message");
          continue;
        }
        sensor::LidarPacket lidar_packet(msg->buf.size());
        memcpy(lidar_packet.buf.data(), msg->buf.data(),
               msg->buf.size());

        lidar_packet_time = m.getTime();
        lidar_packet_handler(lidar_packet);
      }
      // Process IMU packets
      else if (m.getTopic() == "/" + ns_ + "/imu_packets")
      {
        auto msg = m.instantiate<PacketMsg>();
        if (msg)
        {
          sensor::ImuPacket imu_packet(msg->buf.size());
          memcpy(imu_packet.buf.data(), msg->buf.data(),
                 msg->buf.size());

          auto imu_msg = imu_packet_handler(imu_packet);
          std::lock_guard<std::mutex> lock(write_mutex);
          out_bag.write("/" + ns_ + "/imu", m.getTime(), imu_msg);
        }
      }

      // Print progress
      int bar_width = 50;
      float progress = static_cast<float>(count) / total;
      std::cout << "\r[";
      int pos = static_cast<int>(bar_width * progress);
      for (int i = 0; i < bar_width; ++i)
      {
        if (i < pos)
          std::cout << "=";
        else if (i == pos)
          std::cout << ">";
        else
          std::cout << " ";
      }
      std::cout << "] " << int(progress * 100.0) << " %";
      std::cout.flush();
    }

    in_bag.close();
    out_bag.close();
  }

private:
  std::string input_bag_;
  std::string output_bag_;
  std::string ns_;
  std::string metadata_file_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_converter");

  // Usage: bag_converter <input_bag> <output_bag> <ns> [metadata_file]
  if (argc != 4 && argc != 5)
  {
    ROS_ERROR("Usage: bag_converter <input_bag> <output_bag> <ns> [metadata_file]");
    return 1;
  }

  std::string metadata_file = (argc == 5) ? argv[4] : "";
  BagConverter converter(argv[1], argv[2], argv[3], metadata_file);
  converter.process();

  return 0;
}
