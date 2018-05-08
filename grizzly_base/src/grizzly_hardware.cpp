/**
 *
 *  \file
 *  \brief      Class representing Grizzly hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
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

#include "grizzly_base/grizzly_hardware.h"

namespace grizzly_base
{
GrizzlyHardware::GrizzlyHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, grizzly_motor_driver::Interface& interface)
  : nh_(nh), pnh_(pnh), interface_(interface), active_(false)
{
  pnh_.param<double>("gear_ratio", gear_ratio_, 25.0);

  ros::V_string joint_names =
      boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
  std::vector<uint8_t> joint_canids = boost::assign::list_of(3)(4)(5)(6);
  std::vector<float> joint_directions = boost::assign::list_of(1)(-1)(1)(-1);

  for (uint8_t i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints_[i].position, &joints_[i].velocity,
                                                            &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);

    auto driver = std::shared_ptr<grizzly_motor_driver::Driver>(
        new grizzly_motor_driver::Driver(interface_, joint_canids[i], joint_names[i]));
    driver->setGearRatio(gear_ratio_ * joint_directions[i]);
    drivers_.push_back(driver);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  node_.reset(new grizzly_motor_driver::Node(nh_, drivers_));
}

/**
 * Populates the internal joint state struct from the most recent CAN data
 * received from the motor controller.
 *
 * Called from the controller thread.
 */
void GrizzlyHardware::updateJointsFromHardware()
{
  uint8_t index = 0;
  for (const auto& driver : drivers_)
  {
    Joint* f = &joints_[index];
    f->effort = driver->getOutputCurrent();
    f->position = driver->getMeasuredTravel();
    f->velocity = driver->getMeasuredVelocity();
    index++;
  }
}

bool GrizzlyHardware::isActive()
{
  for (const auto& driver : drivers_)
  {
    if (!driver->isRunning())
    {
      active_ = false;
      return false;
    }
  }
  active_ = true;
  return true;
}

bool GrizzlyHardware::anyActive()
{
  for (const auto& driver : drivers_)
  {
    if (driver->isRunning())
    {
      return true;
    }
  }
  return false;
}

bool GrizzlyHardware::isFault()
{
  for (const auto& driver : drivers_)
  {
    if (driver->isFault())
    {
      return true;
    }
  }
  return false;
}

bool GrizzlyHardware::isStopping()
{
  for (const auto& driver : drivers_)
  {
    if (driver->isStopping())
    {
      return true;
    }
  }
  return false;
}

void GrizzlyHardware::requestData()
{
  for (auto& driver : drivers_)
  {
    driver->requestStatus();
    driver->requestFeedback();
  }
}

void GrizzlyHardware::configure()
{
  for (auto& driver : drivers_)
  {
    driver->run();
  }
}
void GrizzlyHardware::reconfigure()
{
  for (auto& driver : drivers_)
  {
    driver->resetState();
  }
}

void GrizzlyHardware::triggerStopping()
{
  for (auto& driver : drivers_)
  {
    driver->setStopping();
  }
}

bool GrizzlyHardware::inReset()
{
  return !active_;
}

void GrizzlyHardware::init()
{
  while (!connectIfNotConnected())
  {
    ros::Duration(1.0).sleep();
  }
}

void GrizzlyHardware::canRead()
{
  // Process all received messages through the connected driver instances.
  grizzly_motor_driver::Frame rx_frame;
  while (interface_.receive(&rx_frame))
  {
    for (auto& driver : drivers_)
    {
      driver->readFrame(rx_frame);
    }
  }
}

void GrizzlyHardware::canSend()
{
  interface_.sendQueued();
}

bool GrizzlyHardware::connectIfNotConnected()
{
  if (!interface_.isConnected())
  {
    if (!interface_.connect())
    {
      ROS_ERROR("Error connecting to motor driver interface. Retrying in 1 second.");
      return false;
    }
    else
    {
      ROS_INFO("Connection to motor driver interface successful.");
    }
  }
  return true;
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void GrizzlyHardware::command()
{
  uint8_t i = 0;
  for (auto& driver : drivers_)
  {
    driver->setSpeed(joints_[i].velocity_command);
    driver->commandSpeed();
    i++;
  }
}

std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> GrizzlyHardware::getDrivers()
{
  return drivers_;
}
}  // namespace grizzly_base
