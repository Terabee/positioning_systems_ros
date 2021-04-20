#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rtls_driver/rtls_driver.hpp"
#include "serial_communication/serial.hpp"
#include "positioning_systems_ros/RtlsTrackerFrame.h"
#include "positioning_systems_ros/RtlsAnchorData.h"

constexpr double MM_TO_M_FACTOR = 0.001;

positioning_systems_ros::RtlsTrackerFrame createTrackerMsgROS(
  const terabee::RtlsDevice::tracker_msg_t& tracker_msg)
{
  positioning_systems_ros::RtlsTrackerFrame msg;
  for (auto anchor : tracker_msg.anchors_data)
  {
    positioning_systems_ros::RtlsAnchorData anchor_msg;
    anchor_msg.id = anchor.id;
    anchor_msg.position.x = MM_TO_M_FACTOR*static_cast<double>(anchor.pos_x);
    anchor_msg.position.y = MM_TO_M_FACTOR*static_cast<double>(anchor.pos_y);
    anchor_msg.position.z = MM_TO_M_FACTOR*static_cast<double>(anchor.pos_z);
    anchor_msg.distance = MM_TO_M_FACTOR*static_cast<double>(anchor.distance);
    msg.anchors.push_back(anchor_msg);
  }

  msg.position.x = MM_TO_M_FACTOR*static_cast<double>(tracker_msg.tracker_position_xyz.at(0));
  msg.position.y = MM_TO_M_FACTOR*static_cast<double>(tracker_msg.tracker_position_xyz.at(1));
  msg.position.z = MM_TO_M_FACTOR*static_cast<double>(tracker_msg.tracker_position_xyz.at(2));
  msg.is_valid_position = tracker_msg.is_valid_position;

  return msg;
}

void publishTransform(const std::string& ref_frame,
                      const std::string& frame,
                      const geometry_msgs::Point& point)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_st;
  transform_st.header.stamp = ros::Time::now();
  transform_st.header.frame_id = ref_frame;
  transform_st.child_frame_id = frame;
  transform_st.transform.translation.x = point.x;
  transform_st.transform.translation.y = point.y;
  transform_st.transform.translation.z = point.z;
  transform_st.transform.rotation.x = 0.0;
  transform_st.transform.rotation.y = 0.0;
  transform_st.transform.rotation.z = 0.0;
  transform_st.transform.rotation.w = 1.0;

  br.sendTransform(transform_st);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtls_tracker_node");
  ros::NodeHandle nh("~");

  bool publish_tf = nh.param("publish_tf", true);
  std::string ref_frame = nh.param("ref_frame", std::string("map"));
  std::string portname = nh.param("portname", std::string("/dev/ttyUSB0"));

  constexpr int serial_timeout = 800;
  std::shared_ptr<terabee::serial_communication::ISerial> serial_port =
      std::make_shared<terabee::serial_communication::Serial>(portname);

  serial_port->setBaudrate(115200);
  serial_port->setTimeout(std::chrono::milliseconds(serial_timeout));

  serial_port->open();

  if (!serial_port->isOpen())
  {
    ROS_ERROR("Failed to open serial port!");
    return 0;
  }

  terabee::RtlsDevice rtls_device(serial_port);
  ros::Publisher rtls_publisher =
    nh.advertise<positioning_systems_ros::RtlsTrackerFrame>("rtls_tracker_frame", 1);

  rtls_device.registerOnDistanceDataCaptureCallback(
    [&rtls_publisher,
     &publish_tf,
     &ref_frame](const terabee::RtlsDevice::tracker_msg_t& tracker_msg)
    {
      ROS_INFO_STREAM("T: " << tracker_msg.timestamp);
      positioning_systems_ros::RtlsTrackerFrame msg;
      msg = createTrackerMsgROS(tracker_msg);
      rtls_publisher.publish(msg);

      if (publish_tf)
      {
        publishTransform(ref_frame, "tracker", msg.position);

        for (auto anchor : msg.anchors)
        {
          std::stringstream anchor_id_hex;
          anchor_id_hex << "0x" << std::setfill ('0')
                        << std::setw(4) << std::hex
                        << anchor.id;
          publishTransform(ref_frame, anchor_id_hex.str(), anchor.position);
        }
      }
    });

  rtls_device.disableTrackerStream();
  serial_port->flushInput();
  rtls_device.requestConfig();
  if (rtls_device.getConfig().type != terabee::RtlsDevice::device_type::tracker)
  {
    ROS_ERROR("Device is not configured as tracker!");
    return 0;
  }
  rtls_device.enableTrackerStream();
  rtls_device.startReadingStream();

  ros::spin();
  rtls_device.stopReadingStream();

  return 0;
}
