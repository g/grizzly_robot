/**
Software License Agreement (BSD)

\file      grizzly_motor_stop.h
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\authors   Mike Hosmar <mhosmar@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

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

#ifndef GRIZZLY_BASE_MOTOR_STOP_PUBLISHER_H
#define GRIZZLY_BASE_MOTOR_STOP_PUBLISHER_H

#include "ros/ros.h"
#include "grizzly_msgs/Status.h"

namespace grizzly_base
{
class MotorStopPublisher
{
public:
  MotorStopPublisher(ros::NodeHandle* nh, double frequency, bool offboard_stop)
    : nh_(nh), frequency_(frequency), offboard_stop_(offboard_stop), enabled_(true)
  {
    pub_ = nh_->advertise<std_msgs::Bool>("mcu/enable_motors", 1);
    timer_pub_ = nh_->createTimer(ros::Duration(1.0 / frequency_), &MotorStopPublisher::pubTimerCallback, this);
    status_timer_ = nh_->createTimer(ros::Duration(1.0 / frequency_), &MotorStopPublisher::mcuTimeoutCallback, this);

    status_sub_ = nh_->subscribe("mcu/status", 1, &MotorStopPublisher::mcuStatusCallback, this);

    if (offboard_stop)
    {
      offboard_sub_ = nh_->subscribe("mcu/offboard_stop", 1, &MotorStopPublisher::externalStopCallback, this);
    }

    last_mcu_update_ = ros::Time::now().toSec();
  }

  void pubTimerCallback(const ros::TimerEvent&)
  {
    msg_.data = enabled_;
    pub_.publish(msg_);
  }

  void mcuTimeoutCallback(const ros::TimerEvent&)
  {
    if ((ros::Time::now().toSec() - last_mcu_update_ ) >= 5.0)
    {
      stopped_ = true;
      ROS_ERROR_THROTTLE(1, "Grizzly lost connection to MCU.");
    }
  }


  void externalStopCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    enabled_ = msg->data;
  }

  void mcuStatusCallback(const grizzly_msgs::Status::ConstPtr& msg)
  {
    stopped_ = msg->stop_engaged;
    if (stopped_)
      has_stopped_ = true;
    last_mcu_update_ = ros::Time::now().toSec();
  }

  bool getStopStatus() const
  {
    return (stopped_);
  }

  bool shouldReset() const
  {
    return (has_stopped_);
  }

  void clearReset()
  {
    has_stopped_ = false;
  }

  void publishStop()
  {
    const std_msgs::Bool msg;
    has_stopped_ =  true;
    pub_.publish(msg);
  }

private:
  ros::NodeHandle* nh_;
  std_msgs::Bool msg_;
  ros::Publisher pub_;
  ros::Subscriber offboard_sub_;
  ros::Subscriber status_sub_;
  ros::Timer timer_pub_;
  ros::Timer status_timer_;
  double frequency_;
  double last_mcu_update_;
  bool offboard_stop_;
  bool enabled_;
  bool stopped_;
  bool has_stopped_;
};

}  // namespace grizzly_base

#endif  // GRIZZLY_BASE_MOTOR_STOP_PUBLISHER_H
