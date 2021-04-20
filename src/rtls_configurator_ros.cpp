#include "rtls_configurator_ros.hpp"

RtlsConfiguratorROS::RtlsConfiguratorROS(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<terabee::RtlsDevice> rtls_device):
  rtls_device_(rtls_device),
  nh_(nh)
{
}

bool RtlsConfiguratorROS::setDevice()
{
  bool success = false;
  std::string device_type_str;
  int priority = 0;

  if (nh_->getParam("device_type", device_type_str) &&
      nh_->getParam("priority", priority))
  {
    terabee::RtlsDevice::device_type device_type;
    if (device_type_str == "Anchor")
      device_type = terabee::RtlsDevice::device_type::anchor;
    else if (device_type_str == "Tracker")
      device_type = terabee::RtlsDevice::device_type::tracker;
    else
      throw std::out_of_range("Invalid device type: " + device_type_str);

    success = rtls_device_->setDevice(device_type, static_cast<uint16_t>(priority));
    if (success)
    {
      ROS_INFO_STREAM("Device type set to: " << device_type_str << " priority: " << priority);
    }
    else
    {
      ROS_ERROR("Failed to set device type and priority!");
    }
  }
  return success;
}

bool RtlsConfiguratorROS::setLabel()
{
  bool success = false;
  int label = 0;

  if (nh_->getParam("label", label))
  {
    success = rtls_device_->setLabel(static_cast<uint16_t>(label));
    if (success)
    {
      ROS_INFO_STREAM("Label set to: " << std::hex << label);
    }
    else
    {
      ROS_ERROR("Failed to configure device label!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setTrackerMessage()
{
  bool success = false;
  std::string tracker_message_mode;

  if (nh_->getParam("tracker_message_mode", tracker_message_mode))
  {
    if (tracker_message_mode == "Short")
      success = rtls_device_->setTrackerMessageShort();
    else if (tracker_message_mode == "Long")
      success = rtls_device_->setTrackerMessageLong();
    else
      throw std::out_of_range("Invalid tracker mode: " + tracker_message_mode);

    if (success)
    {
      ROS_INFO_STREAM("Tracker message type set to: " << tracker_message_mode);
    }
    else
    {
      ROS_ERROR("Failed to set tracker message type!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setNetworkId()
{
  bool success = false;
  int network_id = 0;

  if (nh_->getParam("network_id", network_id))
  {
    success = rtls_device_->setNetworkId(static_cast<uint16_t>(network_id));
    if (success)
    {
      ROS_INFO_STREAM("Network ID set to: " << std::hex << network_id);
    }
    else
    {
      ROS_ERROR("Failed to configure network ID!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setUpdateTime()
{
  bool success = false;
  int update_time_ms = 0;

  if (nh_->getParam("update_time_ms", update_time_ms))
  {
    success = rtls_device_->setUpdateTime(static_cast<uint16_t>(update_time_ms/100));
    if (success)
    {
      ROS_INFO_STREAM("Update time set to: " << update_time_ms);
    }
    else
    {
      ROS_ERROR("Failed to configure update time!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setAnchorPosition()
{
  bool success = false;
  int x = 0;
  int y = 0;
  int z = 0;

  if (nh_->getParam("x", x) &&
      nh_->getParam("y", y) &&
      nh_->getParam("z", z))
  {
    success = rtls_device_->setAnchorPosition(x, y, z);
    if (success)
    {
      ROS_INFO_STREAM("Position set to: (" << x << ", " << y << ", " << z << ").");
    }
    else
    {
      ROS_ERROR("Failed to configure anchor position!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setAnchorHeightForAutoPositioning()
{
  bool success = false;
  int anchor_height = 0;

  if (!nh_->getParam("anchor_height", anchor_height))
  {
    ROS_WARN("Parameter 'anchor_height' not specified.");
  }
  else
  {
    success = rtls_device_->setAnchorHeightForAutoPositioning(anchor_height);
    if (success)
    {
      ROS_INFO_STREAM("Anchor height set to: " << anchor_height);
    }
    else
    {
      ROS_ERROR("Failed to configure anchor height!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setAnchorInitiator()
{
  bool success = false;
  bool is_initiator = false;

  if (!nh_->getParam("is_initiator", is_initiator))
  {
    ROS_WARN("Parameter 'is_initiator' not specified.");
  }
  else
  {
    is_initiator ?
      success = rtls_device_->enableAnchorInitiator() :
      success = rtls_device_->disableAnchorInitiator();
    if (success)
    {
      ROS_INFO_STREAM("Initiator mode " << (is_initiator ? "enabled." : "disabled."));
    }
    else
    {
      ROS_ERROR("Failed to configure initiator mode!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setAutoAnchorPositioning()
{
  bool success = false;
  bool is_auto_anchor_positioning = false;

  if (!nh_->getParam("is_auto_anchor_positioning", is_auto_anchor_positioning))
  {
    ROS_WARN("Parameter 'is_auto_anchor_positioning' not specified.");
  }
  else
  {
    is_auto_anchor_positioning ?
      success = rtls_device_->enableAutoAnchorPositioning() :
      success = rtls_device_->disableAutoAnchorPositioning();

    if (success)
    {
      ROS_INFO_STREAM("Auto anchor positioning " <<
                      (is_auto_anchor_positioning ? "enabled." : "disabled."));
    }
    else
    {
      ROS_ERROR("Failed to configure auto anchor positioning!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setLED()
{
  bool success = false;
  bool is_led_enabled = false;

  if (!nh_->getParam("is_led_enabled", is_led_enabled))
  {
    ROS_WARN("Parameter 'is_led_enabled' not specified.");
  }
  else
  {
    is_led_enabled ?
      success = rtls_device_->enableLED() :
      success = rtls_device_->disableLED();

    if (success)
    {
      ROS_INFO_STREAM("LEDs " << (is_led_enabled ? "enabled." : "disabled."));
    }
    else
    {
      ROS_ERROR("Failed to configure LEDs mode!");
    }
  }

  return success;
}

bool RtlsConfiguratorROS::setTrackerStream()
{
  bool success = false;
  bool is_tracker_stream_mode = false;

  if (!nh_->getParam("is_tracker_stream_mode", is_tracker_stream_mode))
  {
    ROS_WARN("Parameter 'is_tracker_stream_mode' not specified.");
  }
  else
  {
    if (is_tracker_stream_mode)
    {
      success = rtls_device_->enableTrackerStream();
    }
    else
    {
      rtls_device_->disableTrackerStream();
      success = true;
    }

    if (success)
    {
      ROS_INFO_STREAM("Tracker stream mode " <<
                      (is_tracker_stream_mode ? "enabled." : "disabled."));
    }
    else
    {
      ROS_ERROR("Failed to configure tracker stream mode!");
    }
  }

  return success;
}
