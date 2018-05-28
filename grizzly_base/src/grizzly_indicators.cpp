/**
Software License Agreement (BSD)

\file      grizzly_indicators.cpp
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "grizzly_base/grizzly_indicators.h"

namespace grizzly_base
{

GrizzlyIndicators::GrizzlyIndicators(ros::NodeHandle* nh) :
  nh_(nh),
  last_gps_msg_secs_(0),
  first_mcu_msg_received_(false),
  last_mcu_msg_secs_(ros::Time::now().toSec())
{
  // Publishers
  indicators_pub_ = nh_->advertise<grizzly_msgs::Indicators>("mcu/cmd_indicators", 1);

  // Subscribers
  mcu_status_sub_ = nh_->subscribe("mcu/status", 1, &GrizzlyIndicators::mcuStatusCallback, this);
  gps_status_sub_ = nh_->subscribe("fix", 1, &GrizzlyIndicators::gpsStatusCallback, this);
  cmd_vel_sub_ = nh_->subscribe("grizzly_velocity_controller/cmd_vel", 1, &GrizzlyIndicators::cmdVelCallback, this);

  // Timers
  pub_timer_ = nh_->createTimer(ros::Duration(1.0/5.0), &GrizzlyIndicators::timerCb, this);
}

void GrizzlyIndicators::setIndicators(grizzly_msgs::Indicators* indicators, uint8_t indicator_lights[3])
{
    indicators->position_light = indicator_lights[0];
    indicators->autopilot_light = indicator_lights[1];
    indicators->battery_light = indicator_lights[2];
}

void GrizzlyIndicators::mcuStatusCallback(const grizzly_msgs::Status::ConstPtr& mcu_status_msg)
{
  if (!first_mcu_msg_received_)
  {
    first_mcu_msg_received_ = true;
    // Checking Voltage levels from MCU
    if (mcu_status_msg->measured_battery >= BATTERY_FULL_)
      initial_soc_ = 1.0;
    else if (mcu_status_msg->measured_battery <= BATTERY_EMPTY_)
      initial_soc_ = 0.0;
    else
      getSocEstimate(mcu_status_msg->measured_battery);
  }
  else
  {
    mcu_msg_secs_ = ros::Time::now().toSec();
    // P = VI
    instantaneous_power_ = mcu_status_msg->measured_battery * mcu_status_msg->current_battery;
    differential_time_ = mcu_msg_secs_ - last_mcu_msg_secs_;
    // dE = P(t)dt converted to Watt Hours
    differential_energy_consumed_ = (differential_time_ * (instantaneous_power_ + prev_power_)/2.0) / 3600;
    // E2 = E1 + dE
    total_energy_consumed_ = total_energy_consumed_ + differential_energy_consumed_;
    prev_power_ = instantaneous_power_;
    last_mcu_msg_secs_ = mcu_msg_secs_;
  }
}

void GrizzlyIndicators::gpsStatusCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_status_msg)
{
  gps_status_msg_ = *gps_status_msg;
  last_gps_msg_secs_ = ros::Time::now().toSec();
}

void GrizzlyIndicators::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
  cmd_vel_msg_ = *cmd_vel_msg;
}

void GrizzlyIndicators::calculateBatteryLevel()
{
  // Convert SOC value b/w 0 and 1 to equivalent Battery Level with values b/w 0 and 255
  if ((initial_soc_ - total_energy_consumed_/BATTERY_CAPACITY_) <= 0)
    battery_level_ = 0;
  else
    battery_level_ = round((initial_soc_ - total_energy_consumed_/BATTERY_CAPACITY_) * 255);
}

void GrizzlyIndicators::getSocEstimate(double voltage)
{
  initial_soc_ = A1_*exp(-pow((voltage-B1_)/C1_,2)) + A2_*exp(-pow((voltage-B2_)/C2_,2));
}

void GrizzlyIndicators::timerCb(const ros::TimerEvent&)
{
  updateMessage(ros::Time::now().toSec());
  setIndicators(&indicators_msg_, &indicator_lights_[0]);
  indicators_pub_.publish(indicators_msg_);
}

void GrizzlyIndicators::updateMessage(double gps_msg_secs)
{
  // Turn off position lock indicator if no message received within 10s
  if (gps_msg_secs > last_gps_msg_secs_ + GPS_COMMAND_TIMEOUT_SECS_)
  {
    indicator_lights_[0] = grizzly_msgs::Indicators::INDICATOR_OFF;
  }
  else
  {
    // Flash position lock indicator if no fix obtained by GPS
    if (gps_status_msg_.status.status <= 0)
    {
      indicator_lights_[0] = grizzly_msgs::Indicators::INDICATOR_FLASH;
    }
    else
    {
      indicator_lights_[0] = grizzly_msgs::Indicators::INDICATOR_ON;
    }
  }

  // (Temporary) If robot is moving, turn on autopilot light indicator
  if (cmd_vel_msg_.linear.x != 0.0 ||
      cmd_vel_msg_.angular.z != 0.0)
  {
    indicator_lights_[1] = grizzly_msgs::Indicators::INDICATOR_ON;
  }
  else
  {
    indicator_lights_[1] = grizzly_msgs::Indicators::INDICATOR_OFF;
  }

  calculateBatteryLevel();
  indicator_lights_[2] = battery_level_;
}

}  // namespace grizzly_base
