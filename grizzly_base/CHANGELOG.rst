^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grizzly_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2018-06-22)
------------------

0.3.2 (2018-06-21)
------------------

0.3.1 (2018-06-21)
------------------

0.3.0 (2018-06-06)
------------------
* Updated Udev, maintainer and CAN bridge interface.
* [grizzly_base] Added check for MCU timeout.
* [grizzly_base] Fixed cooling logic.
* Remove temp subscriber.
* Fix dependancy on grizzly_motor_msgs. Remove references to puma_motor_msgs.
* Refine fault and estop handling.
* Added runtime fault handling.
* Changes to enable re-enabling motors after estop.
* Fixed static variable issue. First time CM moves wheels. Fixed direction.
* First steps to switch to grizzly motor driver
* Added check to ensure battery_level\_ is not negative
* Added indicator lighting control to PC. Also added estimation of robot power levels from MCU status
* Adding light command control to the PC. Cleaning up some old indigo code. Updating to package format 2.
* Removing commented out packages
* Added UDP rosserial connection and cooling via cmd_vel.
* Return false instead of throwing it, whoops.
* Contributors: Aditya Bhattacharjee, Michael Hosmar, Mike Purvis, Mohamed Elshatshat, Tony Baltovski, abhattacharjee

0.2.1 (2015-10-17)
------------------
* Launchfiles from RS232 comms option.
* Modify MotorFanout to take advantage of active brake.
* Contributors: Mike Purvis

0.2.0 (2015-01-09)
------------------
* Add queue_size to rospy.Publisher.
* Fix catkin includes typo.
* Contributors: Mike Purvis

0.1.1 (2014-02-25)
------------------
* Add missing roboteq dependencies
* Installation of grizzly_base launchers.
* Contributors: Mike Purvis

0.1.0 (2014-02-14)
------------------
* Initial release for Hydro
* Contributors: Mike Purvis
