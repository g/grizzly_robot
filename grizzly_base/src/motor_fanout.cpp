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
    telemetry_timeout_(telemetry_timeout)
  {
    sub_feedback_ = nh_.subscribe("feedback", 1, &RoboteqInterface::feedbackCallback, this);
    pub_cmd_ = nh_.advertise<roboteq_msgs::Command>("cmd", 1);

    brake_timeout_timer_ = nh_.createTimer(ros::Duration(0.11), &RoboteqInterface::brakeTimeoutCallback, this);
    brake_timeout_timer_.stop();
    is_braking_ = false;
    brake_setpoint_ = 0;
  }

  void cmdVelocity(float vel)
  {
    is_braking_ = false;
    roboteq_msgs::Command cmd;
    cmd.mode = roboteq_msgs::Command::MODE_VELOCITY;
    cmd.setpoint = vel;
    pub_cmd_.publish(cmd);
  }

  void cmdBrake()
  {
    // Reset braking timeout.
    brake_timeout_timer_.stop();
    brake_timeout_timer_.start();

    // On the first brake command, we store the current position in order to hold to it.
    // On successive brake commands, we re-issue the command to hold to the previously-stored
    // position.
    if  (!is_braking_)
    {
      if (lastFeedbackValid())
      {
        brake_setpoint_ = last_feedback_->measured_position;
        is_braking_ = true;
      }
      else
      {
        ROS_WARN("Unable to command braking position hold due to lack of up-to-date telemetry.");
        return;
      }
    }

    roboteq_msgs::Command cmd;
    cmd.mode = roboteq_msgs::Command::MODE_POSITION;
    cmd.setpoint = brake_setpoint_;
    pub_cmd_.publish(cmd);
  }

  bool lastFeedbackValid()
  {
    if (!last_feedback_)
    {
      return false;
    }
    ros::Duration age(ros::Time::now() - last_feedback_->header.stamp);
    if (age > telemetry_timeout_)
    {
      throw false;
    }
    return true;
  }

  float getMeasuredVelocity()
  {
    if (lastFeedbackValid())
    {
      return last_feedback_->measured_velocity;
    }
    else
    {
      throw DataTimeout();
    }
  }

protected:
  void brakeTimeoutCallback(const ros::TimerEvent&)
  {
    // No longer braking, so if the brake is re-asserted again, a new brake position
    // for the wheels will be sampled and asserted.
    is_braking_ = false;
  }

  void feedbackCallback(const roboteq_msgs::FeedbackConstPtr& feedback)
  {
    last_feedback_ = feedback;
  }

  roboteq_msgs::FeedbackConstPtr last_feedback_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_feedback_; // , sub_status_;
  ros::Publisher pub_cmd_;
  ros::Duration telemetry_timeout_;

  /**
   * Determines when the brake is no longer being asserted, so that the brake setpoint
   * can be cleared and re-established.
   */
  ros::Timer brake_timeout_timer_;

  /**
   * Records whether vehicle is currently in active braking. When braking is
   * starting, the brake setpoint is sampled from the current state of the
   * motor. Otherwise, it is simply asserted.
   */
  bool is_braking_;

  /**
   * Records position setpoint of vehicle when braking was begun.
   */
  float brake_setpoint_;
};

/**
 */
class MotorFanout
{
public:
  MotorFanout() : nh_(""), gear_ratio_(50.0)
  {
    ros::param::param<double>("~gear_ratio", gear_ratio_, gear_ratio_);

    double telemetry_timeout_secs(0.11), telemetry_period_secs(0.05);
    ros::param::param<double>("~telemetry_timeout", telemetry_timeout_secs, telemetry_timeout_secs);
    ros::param::param<double>("~telemetry_period", telemetry_period_secs, telemetry_period_secs);

    ros::Duration telemetry_period(telemetry_period_secs);
    sub_drive_ = nh_.subscribe("cmd_drive", 1, &MotorFanout::driveCallback, this);
    encoder_timer_ = nh_.createTimer(telemetry_period, &MotorFanout::encodersPublishCallback, this);
    pub_encoders_ = nh_.advertise<grizzly_msgs::Drive>("motors/encoders", 1);

    ros::Duration telemetry_timeout(telemetry_timeout_secs);
    motors.front_left.reset(new RoboteqInterface("motors/front_left", telemetry_timeout));
    motors.front_right.reset(new RoboteqInterface("motors/front_right", telemetry_timeout));
    motors.rear_left.reset(new RoboteqInterface("motors/rear_left", telemetry_timeout));
    motors.rear_right.reset(new RoboteqInterface("motors/rear_right", telemetry_timeout));
  }

protected:
  void encodersPublishCallback(const ros::TimerEvent& timer_event);
  void driveCallback(const grizzly_msgs::DriveConstPtr&);
  static bool nearZero(float val);

  ros::NodeHandle nh_;
  ros::Subscriber sub_drive_;

  /**
   * Governs publish rate of encoder status messages, which are coalated together from
   * the four individual drivers.
   */
  ros::Timer encoder_timer_;
  ros::Publisher pub_encoders_;

  /**
   * Factor between reported motor speed and actual post gearbox wheel speed.
   */
  double gear_ratio_;

  struct {
    boost::shared_ptr<RoboteqInterface> front_left, front_right, rear_left, rear_right;
  } motors;
};

/**
 * Passes through commanded rates of rotation from the Drive message to the
 * individual Roboteq interface nodes.
 */
void MotorFanout::driveCallback(const grizzly_msgs::DriveConstPtr& drive)
{
  if (nearZero(drive->front_left) && nearZero(drive->front_right) &&
      nearZero(drive->rear_left) && nearZero(drive->rear_right))
  {
    ROS_DEBUG("Commanding motors to brake.");
    motors.front_left->cmdBrake();
    motors.front_right->cmdBrake();
    motors.rear_left->cmdBrake();
    motors.rear_right->cmdBrake();
  }
  else
  {
    ROS_DEBUG("Commanding motor velocities: %f %f %f %f",
        drive->front_left, drive->front_right, drive->rear_left, drive->rear_right);
    motors.front_left->cmdVelocity(drive->front_left * gear_ratio_);
    motors.front_right->cmdVelocity(drive->front_right * -gear_ratio_);
    motors.rear_left->cmdVelocity(drive->rear_left * gear_ratio_);
    motors.rear_right->cmdVelocity(drive->rear_right * -gear_ratio_);
  }
}

bool MotorFanout::nearZero(float val)
{
  return std::abs(val) < 0.0001;
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
    encoders.front_left = motors.front_left->getMeasuredVelocity() / gear_ratio_;
    encoders.front_right = motors.front_right->getMeasuredVelocity() / -gear_ratio_;
    encoders.rear_left = motors.rear_left->getMeasuredVelocity() / gear_ratio_;
    encoders.rear_right = motors.rear_right->getMeasuredVelocity() / -gear_ratio_;
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
  ros::init(argc, argv, "~");

  MotorFanout mf;
  ros::spin();
}
