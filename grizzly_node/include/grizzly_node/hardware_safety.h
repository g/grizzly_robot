#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Status.h"
#include "roboteq_msgs/Feedback.h"
#include "grizzly_msgs/RawStatus.h"
#include "std_msgs/Bool.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"


// Motor definitions
#define FR 0
#define FL 1
#define RR 2
#define RL 3

// Diagnostic msg types
#define MOTOR_FAULT_DIAG 0
#define MOTOR_COMM_DIAG 1
#define MCU_COMM_DIAG 2
#define ESTOP_DIAG 3
#define ENC_DIAG 4

// Serious Faults
#define NUM_SER_FAULTS 4
#define NUM_MOTORS 4

class HardwareSafety {
  private:
    ros::NodeHandle node_;
    // Encoder safety characteristics
    double enc_watchdog_period_, enc_error_time_, enc_error_thresh_, enc_tics_diff_thresh_;
    // Physical vehicle parameters
    double width_, gear_down_, wheel_radius_,max_rpm_,rpm_scale
    double roboteq_scale_;

    // watchdog timeouts
    double mcu_watchdog_time_, mot_watchdog_time_;
    double mcu_heartbeat_rxd_,mcu_dead_;
    double estop_status_;
    double precharge_malfunction_;

    // Motor Settings
    double mot_setting_[NUM_MOTORS];
    double encreading_[NUM_MOTORS];
    bool encoder_fault_[NUM_MOTORS];
    double num_enc_fault_[NUM_MOTORS];
    bool mot_hearbeat_rxd_[NUM_MOTORS];
    bool mot_node_dead_[NUM_MOTORS];

    bool motor_fault_[NUM_MOTORS];


    // Publishers
    ros::Publisher cmd_pub_fr_, cmd_pub_fl, cmd_pub_rr, cmd_pub_rl, cmd_estop;
    
    // Subscribers
    ros::Subscriber stat_callback[NUM_MOTORS], feedback_callback[NUM_MOTORS],mcu_status_callback;
      
    // Serious faults
    int serious_fault[NUM_SERIOUS_FAULTS]

    // Diagnostic Packager
    MotionDiag motion_diag(); 

  public:
    HardwareSafety();

};


HardwareSafety::HardwareSafety()
{
  node_.param<double>("enc_watchdog_period_", enc_watchdog_period_,1/10.0);
  node_.param<double>("enc_error_time_", enc_watchdog_period_,1);
  node_.param<double>("enc_error_thresh_", enc_error_thresh_,1000);
  node_.param<double>("enc_tics_diff_thresh_", enc_tics_diff_thresh_,8);

  node_.param<double>("vehicle_width_", width_,1.01);
  node_.param<double>("gearing", gear_down_,50.0);
  node_.param<double>("wheel_radius_", wheel_radius_,0.333);
  node_.param<double>("max_rpm_", max_rpm_,3500.0);
  node_.param<double>("mcu_watchdog_time_", mcu_watchdog_time_, 0.5);
  node_.param<double>("mot_watchdog_time_", mot_watchdog_time_, 2);

  // 1 m/s equals how many RPMs at the wheel?
  rpm_scale = 1
  rpm_scale /= (2*M_PI*wheel_radius_) //convert m/s to rotations at the wheel
  rpm_scale *= gear_down_ //convert rotations at the wheel to rotations at the motor
  rpm_scale *= 60 //seconds to mins

  // Convert rpms to roboteq input units (1000 units to get to max rpm)
  roboteq_scale = rpm_scale * (1000.0/max_rpm);  

  cmd_pub_fr_ = node_.advertise<roboteq_msgs::Command>("motors/front_right/cmd",1);
  cmd_pub_fl_ = node_.advertise<roboteq_msgs::Command>("motors/front_left/cmd",1);
  cmd_pub_rr_ = node_.advertise<roboteq_msgs::Command>("motors/rear_right/cmd",1);
  cmd_pub_rl_ = node_.advertise<roboteq_msgs::Command>("motors/rear_left/cmd",1);
  cmd_estop_ = node_.advertise<std_msgs::Bool>("system_estop",1);

  serious_faults[0] = roboteq_msgs::Status.FAULT_OVERHEAT;
  serious_faults[1] = roboteq_msgs::Status.FAULT_OVERVOLTAGE;
  serious_faults[2] = roboteq_msgs::Status.FAULT_SHORT_CIRCUIT;
  serious_faults[3] = roboteq_msgs::Status.FAULT_MOSFET_FAILURE;

  mcu_heartbeat_rxd_ = false;
  mcu_dead_ = false;
  estop_status_ = grizzly_msgs::RawStatus.ERROR_ESTOP_RESET;
  precharge_malfunction_ = false

  for (int i=0;i<NUM_MOTORS;i++) {
    mot_setting[i] = 0;
    encreading[i] = 0;
    enc_tics_in_error[i] = 0; // track number of consecutive error measurements that were too large
    encoder_fault[i] = false; // assume encoders are okay
    num_enc_fault[i] = 0;
    mot_heartbeat_rxd[i] = false;
    mot_node_dead[i] = true;

    motor_fault[i] = false;
  }
    
  stat_callback[0] = node_.subscribe("motors/front_right/status",

}





class MotionDiag {
  public:
    MotionDiag {


    }
  private:
    

};
















  

}
