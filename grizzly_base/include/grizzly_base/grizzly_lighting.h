/**
Software License Agreement (BSD)

\file      grizzly_lighting.h
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


#ifndef GRIZZLY_BASE_GRIZZLY_LIGHTING_H
#define GRIZZLY_BASE_GRIZZLY_LIGHTING_H

#include<vector>

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "grizzly_msgs/Ambience.h"
#include "grizzly_msgs/Status.h"
#include "grizzly_motor_msgs/MultiStatus.h"

namespace grizzly_base
{

typedef boost::array<uint32_t, 4> pattern;
typedef std::vector<pattern> LightsPatterns;


class GrizzlyLighting
{
public:
  explicit GrizzlyLighting(ros::NodeHandle* nh);

private:
  ros::NodeHandle* nh_;

  ros::Publisher lights_pub_;

  ros::Subscriber user_cmds_sub_;
  ros::Subscriber mcu_status_sub_;
  ros::Subscriber drivers_status_sub_;
  ros::Subscriber cmd_vel_sub_;

  grizzly_motor_msgs::MultiStatus driver_status_msg_;
  grizzly_msgs::Status mcu_status_msg_;
  geometry_msgs::Twist cmd_vel_msg_;

  ros::Timer pub_timer_;
  ros::Timer user_timeout_;
  ros::Timer wakeup_timeout_;

  bool allow_user_;
  bool user_publishing_;
  uint8_t state_;
  uint8_t old_state_;
  uint8_t current_pattern_count_;
  uint32_t current_pattern_[4];

  struct patterns
  {
    LightsPatterns stopped;
    LightsPatterns fault;
    LightsPatterns low_battery;
    LightsPatterns driving;
    LightsPatterns idle;
  }
  patterns_;

  void setLights(grizzly_msgs::Ambience* lights, uint32_t pattern[4]);

  void updateState();
  void updatePattern();

  void userCmdCallback(const grizzly_msgs::Ambience::ConstPtr& lights_msg);
  void mcuStatusCallback(const grizzly_msgs::Status::ConstPtr& status_msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void driversStatusCallback(const grizzly_motor_msgs::MultiStatus::ConstPtr& status_msg);
  void timerCb(const ros::TimerEvent&);
  void userTimeoutCb(const ros::TimerEvent&);
  void wakeupTimeoutCb(const ros::TimerEvent&);
};

}  // namespace grizzly_base

#endif  // GRIZZLY_BASE_GRIZZLY_LIGHTING_H
