/**
Software License Agreement (BSD)

\file      grizzly_indicators.h
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


#ifndef GRIZZLY_BASE_GRIZZLY_INDICATORS_H
#define GRIZZLY_BASE_GRIZZLY_INDICATORS_H

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "grizzly_msgs/Indicators.h"
#include "grizzly_msgs/Status.h"
#include "sensor_msgs/NavSatFix.h"

namespace grizzly_base
{

class GrizzlyIndicators
{
public:
  explicit GrizzlyIndicators(ros::NodeHandle* nh);

private:
  ros::NodeHandle* nh_;

  // Publishers
  ros::Publisher indicators_pub_;

  // Subscribers
  ros::Subscriber mcu_status_sub_;
  ros::Subscriber gps_status_sub_;
  ros::Subscriber cmd_vel_sub_;

  // Messages
  grizzly_msgs::Indicators indicators_msg_;
  sensor_msgs::NavSatFix gps_status_msg_;
  geometry_msgs::Twist cmd_vel_msg_;

  // Timers
  ros::Timer pub_timer_;

  uint8_t battery_level_;
  uint8_t indicator_lights_[3];
  double last_gps_msg_secs_;

  static constexpr double GPS_COMMAND_TIMEOUT_SECS_ = 10;

  void setIndicators(grizzly_msgs::Indicators* indicators, uint8_t indicator_lights[3]);
  void updateMessage(double gps_msg_secs);

  // Callbacks
  void mcuStatusCallback(const grizzly_msgs::Status::ConstPtr& status_msg);
  void gpsStatusCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void timerCb(const ros::TimerEvent&);

  // Variables and Functions for Battery Level calculations
  static constexpr double BATTERY_FULL_ = 50.0;
  static constexpr double BATTERY_EMPTY_ = 46.5;
  static constexpr double BATTERY_CAPACITY_ = 105*48*2; // 105Ah * 48V * 2 Battery Packs
  // Constants used for initial SOC calculation
  static constexpr double A1_ = 0.5256, B1_ = 50.09, C1_ = 0.0485,
                          A2_ = 0.8292, B2_ = 51.94, C2_ = 2.822;

  bool first_mcu_msg_received_;
  double instantaneous_power_;
  double differential_time_;
  double differential_energy_consumed_;
  double total_energy_consumed_;
  double prev_power_;
  double last_mcu_msg_secs_;
  double mcu_msg_secs_;
  double initial_soc_;

  void calculateBatteryLevel();
  void getSocEstimate(double voltage);

};

}  // namespace grizzly_base

#endif  // GRIZZLY_BASE_GRIZZLY_INDICATORS_H
