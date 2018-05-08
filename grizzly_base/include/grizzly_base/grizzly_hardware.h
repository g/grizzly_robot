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

#ifndef GRIZZLY_BASE_GRIZZLY_HARDWARE_H
#define GRIZZLY_BASE_GRIZZLY_HARDWARE_H

#include <vector>
#include <thread>
#include "boost/assign.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/frame.h"
#include "grizzly_motor_driver/interface.h"
#include "grizzly_motor_driver/node.h"
#include "grizzly_motor_driver/diagnostic_updater.h"

#include "std_msgs/Float64.h"

namespace grizzly_base
{
class GrizzlyHardware : public hardware_interface::RobotHW
{
public:
  GrizzlyHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, grizzly_motor_driver::Interface& gateway);
  void init();                   // Connect to CAN
  bool connectIfNotConnected();  // Keep trying till it connects
  std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> getDrivers();
  void configure();  // Configures the motor drivers
  void reconfigure();
  void triggerStopping();
  bool isActive();
  bool anyActive();
  bool isFault();
  bool isStopping();

  bool inReset();           // Returns if the cm should be reset based on the state of the motors drivers.
                            // If they have been configured.
  void requestData();
  void updateJointsFromHardware();
  void command();

  void canSend();
  void canRead();

private:
  ros::NodeHandle nh_, pnh_;

  grizzly_motor_driver::Interface& interface_;
  std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> drivers_;
  std::shared_ptr<grizzly_motor_driver::Node> node_;

  bool active_;
  double gear_ratio_;
  int encoder_cpr_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  } joints_[4];
};

}  // namespace grizzly_base

#endif  // GRIZZLY_BASE_GRIZZLY_HARDWARE_H
