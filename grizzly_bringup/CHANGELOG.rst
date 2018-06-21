^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grizzly_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2018-06-21)
------------------
* Sensor addition (`#11 <https://github.com/g/grizzly_robot/issues/11>`_)
  * Updated most of the payload launches
  * Changed default ptu port for udev
* Contributors: dniewinski

0.3.1 (2018-06-21)
------------------
* [grizzly_bringup] Removed install for non-existent file.
* Contributors: Tony Baltovski

0.3.0 (2018-06-06)
------------------
* Updated Udev, maintainer and CAN bridge interface.
* Added install script for can up script.  Added provider to add network depend to systemd service.
* Adding light command control to the PC. Cleaning up some old indigo code. Updating to package format 2.
* Removing commented out packages
* Cleaning up merge issues that I missed
* Added UDP rosserial connection and cooling via cmd_vel.
* Contributors: Michael Hosmar, Mohamed Elshatshat, Tony Baltovski

0.2.1 (2015-10-17)
------------------

0.2.0 (2015-01-09)
------------------
* Update dependencies, new-style install script.
* Add line to install launchers from grizzly_navigation.
* Remove ekf launcher and config; this is now in grizzly_navigation.
* Remove udev rule installation.
* Move udev rules to debian folder.
* Add robot_upstart dependency
* Contributors: Mike Purvis

0.1.1 (2014-02-25)
------------------
* Add robot_pose_ekf as dependency
* Add diagnostic_aggregator and enu dependencies
* Add description and teleop
* Add robot_upstart as dependency.
* Contributors: Mike Purvis

0.1.0 (2014-02-14)
------------------
* Initial Hydro release.
* Contributors: Mike Irvine, Mike Purvis, Prasenjit Mukherjee, Ryan Gariepy
