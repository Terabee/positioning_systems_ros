#include <ros/ros.h>
#include "serial_communication/serial.hpp"
#include "rtls_driver/rtls_driver.hpp"
#include "rtls_configurator_ros.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtls_device_config_node");
  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");

  std::string portname = nh->param("portname", std::string("/dev/ttyUSB0"));

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

  auto rtls_device = std::make_shared<terabee::RtlsDevice>(serial_port);
  RtlsConfiguratorROS rtls_configurator(nh, rtls_device);

  rtls_device->disableTrackerStream();
  serial_port->flushInput();

  rtls_configurator.setDevice();
  rtls_configurator.setTrackerMessage();
  rtls_configurator.setLabel();
  rtls_configurator.setNetworkId();
  rtls_configurator.setUpdateTime();
  rtls_configurator.setAnchorPosition();
  rtls_configurator.setAnchorHeightForAutoPositioning();
  rtls_configurator.setAnchorInitiator();
  rtls_configurator.setAutoAnchorPositioning();
  rtls_configurator.setLED();
  rtls_configurator.setTrackerStream();

  serial_port->flushInput();
  rtls_device->requestConfig();
  auto rtls_config = rtls_device->getConfig();
  std::string config = rtls_device->serializeConfig();
  ROS_INFO_STREAM(config);
  return 0;
}
