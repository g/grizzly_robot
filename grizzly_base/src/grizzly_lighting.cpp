/**
Software License Agreement (BSD)

\file      grizzly_lighting.cpp
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

#include "grizzly_base/grizzly_lighting.h"
#include <boost/assign/list_of.hpp>

namespace grizzly_base
{

namespace States
{
  enum State
  {
    Idle = 0,
    Driving,
    Charged,
    Charging,
    LowBattery,
    NeedsReset,
    Fault,
    Stopped,
    NumberOfStates
  };
}  // namespace States
typedef States::State State;

namespace Intensities
{
  enum Intensity
  {
    Off = 0x00,
    High = 0xFF,
    MedHigh = 0xAA,
    Med = 0x5A,
    MedLow = 0x1F,
    Low = 0x05
  };
}  // namespace Intensities
typedef Intensities::Intensity Intensity;


GrizzlyLighting::GrizzlyLighting(ros::NodeHandle* nh) :
  nh_(nh),
  allow_user_(false),
  user_publishing_(false),
  state_(States::Idle),
  current_pattern_count_(0)
{
  lights_pub_ = nh_->advertise<grizzly_msgs::Ambience>("mcu/cmd_ambience", 1);

  user_cmds_sub_ = nh_->subscribe("mcu/cmd_ambience", 1, &GrizzlyLighting::userCmdCallback, this);
  mcu_status_sub_ = nh_->subscribe("mcu/status", 1, &GrizzlyLighting::mcuStatusCallback, this);
  // puma_status_sub_ = nh_->subscribe("status", 1, &GrizzlyLighting::pumaStatusCallback, this);
  cmd_vel_sub_ = nh_->subscribe("cmd_vel", 1, &GrizzlyLighting::cmdVelCallback, this);

  pub_timer_ = nh_->createTimer(ros::Duration(1.0/5), &GrizzlyLighting::timerCb, this);
  user_timeout_ = nh_->createTimer(ros::Duration(1.0/1), &GrizzlyLighting::userTimeoutCb, this);
  wakeup_timeout_ = nh_->createTimer(ros::Duration(5.2/1), &GrizzlyLighting::wakeupTimeoutCb, this, true);

  using namespace Intensities;  // NOLINT(build/namespaces)
  patterns_.stopped.push_back(boost::assign::list_of(High)(High)(High)(High));
  patterns_.stopped.push_back(boost::assign::list_of(Off)(Off)(Off)(Off));

  patterns_.fault.push_back(boost::assign::list_of(Off)(High)(Off)(High));
  patterns_.fault.push_back(boost::assign::list_of(High)(Off)(High)(Off));

  patterns_.reset.push_back(boost::assign::list_of(High)(High)(Off)(Off));
  patterns_.reset.push_back(boost::assign::list_of(Off)(Off)(High)(High));
  
  patterns_.low_battery.push_back(
    boost::assign::list_of(Low)(Low)(Low)(Low));
  patterns_.low_battery.push_back(
    boost::assign::list_of(MedLow)(MedLow)(MedLow)(MedLow));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Med)(Med)(Med)(Med));
  patterns_.low_battery.push_back(
    boost::assign::list_of(MedHigh)(MedHigh)(MedHigh)(MedHigh));
  patterns_.low_battery.push_back(
    boost::assign::list_of(High)(High)(High)(High));
  patterns_.low_battery.push_back(
    boost::assign::list_of(MedHigh)(MedHigh)(MedHigh)(MedHigh));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Med)(Med)(Med)(Med));
  patterns_.low_battery.push_back(
    boost::assign::list_of(MedLow)(MedLow)(MedLow)(MedLow));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Low)(Low)(Low)(Low));

  patterns_.charged.push_back(
    boost::assign::list_of(High)(High)(High)(High));

  patterns_.charging.push_back(
    boost::assign::list_of(High)(Off)(Off)(Off));
  patterns_.charging.push_back(
    boost::assign::list_of(Off)(High)(Off)(Off));
  patterns_.charging.push_back(
    boost::assign::list_of(Off)(Off)(High)(Off));
  patterns_.charging.push_back(
    boost::assign::list_of(Off)(Off)(Off)(High));

  patterns_.driving.push_back(boost::assign::list_of(High)(High)(High)(High));

  patterns_.idle.push_back(boost::assign::list_of(Low)(Low)(Low)(Low));

  for(int i = 0; i < 13; i++)
  {
  patterns_.idle.push_back(
    boost::assign::list_of(Low + i*20)(Low + i*20)(Low + i*20)(Low + i*20));
  }
  for(int j = 0; j < 13; j++)
  {
  patterns_.idle.push_back(
    boost::assign::list_of(High - j*20)(High - j*20)(High - j*20)(High - j*20));
  }

}

void GrizzlyLighting::setLights(grizzly_msgs::Ambience* lights, uint32_t pattern[4])
{
  for (int i = 0; i < 4; i++)
  {
    lights->body_lights[i] = pattern[i];
  }
}

void GrizzlyLighting::userCmdCallback(const grizzly_msgs::Ambience::ConstPtr& lights_msg)
{
  if (allow_user_)
  {
    lights_pub_.publish(lights_msg);
  }
  user_publishing_ = true;
}

void GrizzlyLighting::mcuStatusCallback(const grizzly_msgs::Status::ConstPtr& status_msg)
{
  mcu_status_msg_ = *status_msg;
}

// void GrizzlyLighting::pumaStatusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
// {
//   pumas_status_msg_ = *status_msg;
// }

void GrizzlyLighting::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_msg_ = *msg;
}

void GrizzlyLighting::timerCb(const ros::TimerEvent&)
{
  updateState();

  grizzly_msgs::Ambience lights_msg;
  updatePattern();
  setLights(&lights_msg, &current_pattern_[0]);
  lights_pub_.publish(lights_msg);
}

void GrizzlyLighting::userTimeoutCb(const ros::TimerEvent&)
{
  user_publishing_ = false;
}

void GrizzlyLighting::wakeupTimeoutCb(const ros::TimerEvent&)
{
  patterns_.idle.clear();
  patterns_.idle.push_back(boost::assign::list_of(0x05)(0x05)(0x05)(0x05)); // After wakeup finishes set idle pattern to "Low" 
}

//TODO: Add fault check based on motor controller status message.
void GrizzlyLighting::updateState()
{
  if (mcu_status_msg_.external_stop_engaged == true)
  {
    state_ = States::Stopped;
  }
  // else if (mcu_status_msg_.drivers_active == false)
  // {
  //   state_ = States::NeedsReset;
  // }
  // else if (pumas_status_msg_.drivers.size() == 4 &&
  //         (pumas_status_msg_.drivers[0].fault != 0 ||
  //          pumas_status_msg_.drivers[1].fault != 0 ||
  //          pumas_status_msg_.drivers[2].fault != 0 ||
  //          pumas_status_msg_.drivers[3].fault != 0))
  // {
  //   state_ = States::Fault;
  // }
  else if (mcu_status_msg_.measured_battery <= 24.0)
  {
    state_ = States::LowBattery;
  }
  else if (cmd_vel_msg_.linear.x != 0.0 ||
           cmd_vel_msg_.linear.y != 0.0 ||
           cmd_vel_msg_.angular.z != 0.0)
  {
    state_ = States::Driving;
  }
  else
  {
    state_ = States::Idle;
  }
}

void GrizzlyLighting::updatePattern()
{
  if (old_state_ != state_)
  {
    current_pattern_count_ = 0;
  }

  switch (state_)
  {
    case States::Stopped:
      if (current_pattern_count_ >= patterns_.stopped.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.stopped[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Fault:
      if (current_pattern_count_ >= patterns_.fault.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.fault[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::NeedsReset:
      if (current_pattern_count_ >= patterns_.reset.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.reset[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::LowBattery:
      if (current_pattern_count_ >= patterns_.low_battery.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.low_battery[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Charged:
      if (current_pattern_count_ >= patterns_.charged.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.charged[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Charging:
      if (current_pattern_count_ >= patterns_.charging.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.charging[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Driving:
      if (current_pattern_count_ >= patterns_.driving.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.driving[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Idle:
      if (current_pattern_count_ >= patterns_.idle.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.idle[current_pattern_count_], sizeof(current_pattern_));
      break;
  }
  old_state_ = state_;
  current_pattern_count_++;
}

}  // namespace grizzly_base
