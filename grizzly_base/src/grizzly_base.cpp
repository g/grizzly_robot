/**
 *
 *  \file
 *  \brief      Main entry point for grizzly base.
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

#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include "controller_manager/controller_manager.h"
#include "grizzly_base/grizzly_diagnostic_updater.h"
#include "grizzly_base/grizzly_hardware.h"
#include "grizzly_base/grizzly_cooling.h"
#include "grizzly_base/grizzly_lighting.h"
#include "grizzly_base/grizzly_indicators.h"
#include "grizzly_base/passive_joint_publisher.h"
#include "grizzly_motor_driver/diagnostic_updater.h"
#include "grizzly_base/grizzly_motor_stop.h"
#include "ros/ros.h"
#include "rosserial_server/udp_socket_session.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, grizzly_base::GrizzlyHardware* robot, controller_manager::ControllerManager* cm,
                   grizzly_base::MotorStopPublisher* motor_stop)
{
  time_source::time_point last_time = time_source::now();
  ros::Duration estop_delay(0);
  bool was_active = false;

  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    if (robot->isFault())
    {
      motor_stop->publishStop();
      robot->triggerStopping();
    }
    else if (motor_stop->getStopStatus())
    {
      robot->triggerStopping();
    }
    else if (!robot->isStopping() && motor_stop->shouldReset() && robot->anyActive())
    {
      robot->triggerStopping();
      motor_stop->clearReset();
    }

    if (robot->isStopping())
    {
      if (estop_delay.isZero())
      {
        estop_delay += elapsed;
      }
      else if (!estop_delay.isZero())
      {
        if (estop_delay >= ros::Duration(3))
        {
          estop_delay = ros::Duration(0);
          robot->reconfigure();
          motor_stop->clearReset();
        }
        else
        {
          estop_delay += elapsed;
        }
      }
    }

    if (robot->isActive())
    {
      robot->updateJointsFromHardware();
    }
    if (!robot->isActive())
    {
      robot->configure();
      if (!robot->anyActive())
      {
        was_active = false;
      }
    }

    robot->canSend();

    cm->update(ros::Time::now(), elapsed, robot->inReset());

    if (robot->isActive())
    {
      robot->command();
      robot->requestData();
    }
    if (robot->anyActive() && !was_active)
    {
      motor_stop->clearReset();  // clear estops just after configuring
      was_active = true;
    }

    robot->canSend();
    rate.sleep();
  }
}

void canReadThread(ros::Rate rate, grizzly_base::GrizzlyHardware* robot)
{
  while (1)
  {
    robot->canRead();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "grizzly_node");
  ros::NodeHandle nh, pnh("~");

  // Create the socket rosserial server in a background ASIO event loop.
  boost::asio::io_service io_service;

  // Connect to the MCU using rosserial_server_upd.
  new rosserial_server::UdpSocketSession(io_service, udp::endpoint(address::from_string("192.168.131.1"), 11411),
                                         udp::endpoint(address::from_string("192.168.131.2"), 11411));
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  std::string canbus_dev;
  pnh.param<std::string>("canbus_dev", canbus_dev, "vcan0");
  grizzly_motor_driver::Interface interface(canbus_dev);

  grizzly_base::GrizzlyHardware grizzly(nh, pnh, interface);

  // Configure the CAN connection
  grizzly.init();
  // Create a thread to start reading can messages.
  std::thread canReadT(&canReadThread, ros::Rate(200), &grizzly);

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&grizzly, controller_nh);
  // This provides the ability to use a off-board software base stop for the base platform.
  bool external_stop;
  pnh.param<bool>("offboard_stop_control", external_stop, false);
  grizzly_base::MotorStopPublisher motor_stop_publisher(&nh, 10, external_stop);
  std::thread controlT(&controlThread, ros::Rate(25), &grizzly, &cm, &motor_stop_publisher);

  // Lighting control.
  grizzly_base::GrizzlyLighting lighting(&nh);

  // Indicators control
  grizzly_base::GrizzlyIndicators indicators(&nh);

  // Create diagnostic updater, to update itself on the ROS thread.
  grizzly_base::GrizzlyDiagnosticUpdater grizzly_diagnostic_updater;
  grizzly_motor_driver::GrizzlyMotorDriverDiagnosticUpdater grizzly_motor_driver_diagnostic_updater;

  // Joint state publisher for passive front axle.
  ros::V_string passive_joint = boost::assign::list_of("front_rocker");
  grizzly_base::PassiveJointPublisher passive_joint_publisher(nh, passive_joint, 50);

  // Cooling control for the fans.
  grizzly_base::GrizzlyCooling cooling(&nh);

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}
