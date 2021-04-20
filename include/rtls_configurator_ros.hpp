#ifndef RTLS_CONFIGURATOR_ROS_HPP
#define RTLS_CONFIGURATOR_ROS_HPP

#include <ros/ros.h>
#include "rtls_driver/rtls_driver.hpp"

class RtlsConfiguratorROS
{
public:
  explicit RtlsConfiguratorROS(std::shared_ptr<ros::NodeHandle> nh,
                               std::shared_ptr<terabee::RtlsDevice> rtls_device);
  ~RtlsConfiguratorROS() = default;
  RtlsConfiguratorROS() = delete;
  RtlsConfiguratorROS(const RtlsConfiguratorROS&) = delete;
  RtlsConfiguratorROS(RtlsConfiguratorROS &&) = delete;

  bool setDevice();
  bool setTrackerMessage();
  bool setLabel();
  bool setNetworkId();
  bool setUpdateTime();
  bool setAnchorPosition();
  bool setAnchorHeightForAutoPositioning();
  bool setAnchorInitiator();
  bool setAutoAnchorPositioning();
  bool setLED();
  bool setTrackerStream();

private:
  std::shared_ptr<terabee::RtlsDevice> rtls_device_;
  std::shared_ptr<ros::NodeHandle> nh_;
};

#endif // RTLS_CONFIGURATOR_ROS_HPP
