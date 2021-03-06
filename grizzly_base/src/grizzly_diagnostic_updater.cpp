/**
 *
 *  \file
 *  \brief      Diagnostic updating class for Grizzly
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <unistd.h>

#include "boost/algorithm/string/predicate.hpp"
#include "diagnostic_updater/update_functions.h"
#include "grizzly_base/grizzly_diagnostic_updater.h"

namespace grizzly_base
{

GrizzlyDiagnosticUpdater::GrizzlyDiagnosticUpdater()
{
  setHardwareID("unknown");
  gethostname(hostname_, 1024);

  add("General", this, &GrizzlyDiagnosticUpdater::generalDiagnostics);
  add("Battery", this, &GrizzlyDiagnosticUpdater::batteryDiagnostics);
  add("User voltage supplies", this, &GrizzlyDiagnosticUpdater::voltageDiagnostics);
  add("Current consumption", this, &GrizzlyDiagnosticUpdater::currentDiagnostics);
  add("Power consumption", this, &GrizzlyDiagnosticUpdater::powerDiagnostics);
  add("Temperature", this, &GrizzlyDiagnosticUpdater::temperatureDiagnostics);

  // The arrival of this message runs the update() method and triggers the above callbacks.
  status_sub_ = nh_.subscribe("mcu/status", 5, &GrizzlyDiagnosticUpdater::statusCallback, this);

  // These message frequencies are reported on separately.
  ros::param::param("~expected_imu_frequency", expected_imu_frequency_, 50.0);
  imu_diagnostic_ = new diagnostic_updater::TopicDiagnostic("/imu/data_raw", *this,
      diagnostic_updater::FrequencyStatusParam(&expected_imu_frequency_, &expected_imu_frequency_, 0.15),
      diagnostic_updater::TimeStampStatusParam(-1, 1.0));
  imu_sub_ = nh_.subscribe("/imu/data_raw", 5, &GrizzlyDiagnosticUpdater::imuCallback, this);

  // Publish whether the wireless interface has an IP address every second.
  ros::param::param<std::string>("~wireless_interface", wireless_interface_, "wlan0");
  ROS_INFO_STREAM("Checking for wireless connectivity on interface: " << wireless_interface_);
  wifi_connected_pub_ = nh_.advertise<std_msgs::Bool>("wifi_connected", 1);
  wireless_monitor_timer_ =
    nh_.createTimer(ros::Duration(1.0), &GrizzlyDiagnosticUpdater::wirelessMonitorCallback, this);
}

void GrizzlyDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.addf("MCU uptime", "%d seconds", static_cast<uint32_t>(last_status_->mcu_uptime.toSec()));
  stat.add("External stop status", last_status_->external_stop_engaged ? "present" : "absent");
  stat.add("Run/stop status", last_status_->stop_engaged ? "stopped" : "running");

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System OK.");
}

void GrizzlyDiagnosticUpdater::batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Battery Voltage (V)", last_status_->measured_battery);

  if (last_status_->measured_battery > 60.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
  }
  else if (last_status_->measured_battery < 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
  }
  else if (last_status_->measured_battery < 46.5)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery critically under voltage.");
  }
  else if (last_status_->measured_battery < 48.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery low voltage.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
  }
}

void GrizzlyDiagnosticUpdater::voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("12V Supply (V)", last_status_->measured_12v);

  if (last_status_->measured_12v > 12.5)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "User supply overvoltage. Accessories may be damaged.");
  }
  else if (last_status_->measured_12v < 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "User supplies absent. Check tray fuses.");
  }
  else if (last_status_->measured_12v < 11.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Voltage supplies undervoltage. Check loading levels.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "User supplies OK.");
  }
}

void GrizzlyDiagnosticUpdater::currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Total current (A)", last_status_->current_battery);

  if (last_status_->current_battery > 32.0)
  {
     stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Current draw critical.");
  }
  else if (last_status_->current_battery > 20.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current draw warning.");
  }
  else if (last_status_->current_battery > 10.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw requires monitoring.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw nominal.");
  }
}

void GrizzlyDiagnosticUpdater::powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Total power consumption (Wh)", last_status_->total_power_consumed);

  if (last_status_->total_power_consumed > 260.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed exceeds capacity of standard battery.");
  }
  else if (last_status_->total_power_consumed > 220.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed approaches capacity of standard battery.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
  }
}

void GrizzlyDiagnosticUpdater::temperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("MCU temperature (C)", last_status_->mcu_temperature);

  if (last_status_->mcu_temperature > 100.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "MCU temperature too HOT.");
  }
  else if (last_status_->mcu_temperature > 60.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "MCU temperature getting warm.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "MCU temperature OK.");
  }
}

void GrizzlyDiagnosticUpdater::statusCallback(const grizzly_msgs::Status::ConstPtr& status)
{
  // Fresh data from the MCU, publish a diagnostic update.
  last_status_ = status;
  setHardwareID(hostname_ + '-' + last_status_->hardware_id);
  update();
}

void GrizzlyDiagnosticUpdater::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_diagnostic_->tick(msg->header.stamp);
}

void GrizzlyDiagnosticUpdater::wirelessMonitorCallback(const ros::TimerEvent& te)
{
  std_msgs::Bool wifi_connected_msg;
  wifi_connected_msg.data = false;

  // Get system structure of interface IP addresses.
  struct ifaddrs* ifa_head;
  if (getifaddrs(&ifa_head) != 0)
  {
    ROS_WARN("System call getifaddrs returned error code. Unable to detect network interfaces.");
    return;
  }

  // Iterate structure looking for the wireless interface.
  struct ifaddrs* ifa_current = ifa_head;
  while (ifa_current != NULL)
  {
    if (strcmp(ifa_current->ifa_name, wireless_interface_.c_str()) == 0)
    {
      int family = ifa_current->ifa_addr->sa_family;
      if (family == AF_INET || family == AF_INET6)
      {
        wifi_connected_msg.data = true;
        break;
      }
    }

    ifa_current = ifa_current->ifa_next;
  }

  // Free structure, publish result message.
  freeifaddrs(ifa_head);
  wifi_connected_pub_.publish(wifi_connected_msg);
}

}  // namespace grizzly_base

