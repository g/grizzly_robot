/**
Software License Agreement (BSD)

\file      motor_fanout.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

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

#include "ros/ros.h"
#include "grizzly_msgs/Drive.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Feedback.h"

#include <boost/foreach.hpp>
#include <stdexcept>

class DataTimeout : public std::runtime_error
{
public:
  DataTimeout(ros::Duration age) :
    runtime_error("Data exceeded maximum specified age.") {}
  DataTimeout() :
    runtime_error("Data exceeded maximum specified age.") {}
};

class RoboteqInterface
{
public:
  RoboteqInterface(std::string ns, ros::Duration telemetry_timeout) :
    nh_(ns),
    telemetry_timeout_(telemetry_timeout),
    // sub_status_(nh_.subscribe("status", 1, &RoboteqInterface::status_callback, this)),
    sub_feedback_(nh_.subscribe("feedback", 1, &RoboteqInterface::feedbackCallback, this)),
    pub_cmd_(nh_.advertise<roboteq_msgs::Command>("cmd", 1))
  {
  }

  void cmdVelocity(float vel)
  {
    roboteq_msgs::Command cmd;
    cmd.commanded_velocity = vel; 
    pub_cmd_.publish(cmd);
  }

  float getMeasuredVelocity()
  {
    if (!last_feedback_)
    {
      throw DataTimeout();
    }
    ros::Duration age(ros::Time::now() - last_feedback_->header.stamp);
    if (age > telemetry_timeout_)
    {
      throw DataTimeout(age);
    }
    return last_feedback_->measured_velocity;
  }

protected: 
  void feedbackCallback(const roboteq_msgs::FeedbackConstPtr& feedback)
  {
    last_feedback_ = feedback;
  }

  roboteq_msgs::FeedbackConstPtr last_feedback_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_feedback_; // , sub_status_;
  ros::Publisher pub_cmd_;
  ros::Duration telemetry_timeout_;
};

/**
 */
class MotorFanout
{
public:
  MotorFanout(double gear_ratio, ros::Duration telemetry_period, ros::Duration telemetry_timeout) :
    nh_(""),
    sub_drive_(nh_.subscribe("cmd_drive", 1, &MotorFanout::driveCallback, this)),
    encoder_timer_(nh_.createTimer(telemetry_period, &MotorFanout::encodersPublishCallback, this)),
    pub_encoders_(nh_.advertise<grizzly_msgs::Drive>("encoders", 1)),
    front_left("front_left", telemetry_timeout),
    front_right("front_right", telemetry_timeout),
    rear_left("rear_left", telemetry_timeout),
    rear_right("rear_right", telemetry_timeout)
  { 
  }

protected:
  void encodersPublishCallback(const ros::TimerEvent& timer_event);
  void driveCallback(const grizzly_msgs::DriveConstPtr&);

  ros::NodeHandle nh_;
  ros::Subscriber sub_drive_;
  ros::Timer encoder_timer_;
  ros::Publisher pub_encoders_;
  double gear_ratio_;

  RoboteqInterface front_left, front_right, rear_left, rear_right;
};

/**
 * Passes through commanded rates of rotation from the Drive message to the
 * individual Roboteq interface nodes.
 */
void MotorFanout::driveCallback(const grizzly_msgs::DriveConstPtr& drive)
{
  front_left.cmdVelocity(drive->front_left * gear_ratio_);
  front_right.cmdVelocity(drive->front_right * gear_ratio_);
  rear_left.cmdVelocity(drive->rear_left * gear_ratio_);
  rear_right.cmdVelocity(drive->rear_right * gear_ratio_);
}

/**
 * For now, this is very naive; no attempt at all is made to synchronize with the incoming
 * Roboteq messages. A possible future implementation could examine the time gaps between
 * the four incoming topics, and then decide to trigger publication of the output message
 * on whichever feedback message arrives "last" of the group. */
void MotorFanout::encodersPublishCallback(const ros::TimerEvent& timer_event)
{
  grizzly_msgs::Drive encoders;
  encoders.header.stamp = ros::Time::now();
  try
  {
    encoders.front_left = front_left.getMeasuredVelocity() / gear_ratio_;
    encoders.front_right = front_right.getMeasuredVelocity() / gear_ratio_;
    encoders.rear_left = rear_left.getMeasuredVelocity() / gear_ratio_;
    encoders.rear_right = rear_right.getMeasuredVelocity() / gear_ratio_;
    pub_encoders_.publish(encoders);
  }
  catch (DataTimeout)
  {
    ROS_WARN_THROTTLE(10, "Encoder data from motor drivers missing or too delayed to republish.");
  }
}

/**
 * Node entry point.
 */
int main (int argc, char ** argv)
{
  ros::init(argc, argv, "grizzly_motor_fanout"); 

  double gear_ratio(50.0), telemetry_timeout(0.03), telemetry_period(0.02);
  ros::param::param<double>("~gear_ratio", gear_ratio, gear_ratio);
  ros::param::param<double>("~telemetry_timeout", telemetry_timeout, telemetry_timeout);
  ros::param::param<double>("~telemetry_period", telemetry_period, telemetry_period);
  
  MotorFanout mf(gear_ratio, ros::Duration(telemetry_period), ros::Duration(telemetry_timeout));
  ros::spin();
}
