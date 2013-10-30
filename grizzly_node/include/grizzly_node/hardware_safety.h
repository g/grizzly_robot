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
    int pre_charge_timeout;


    // Publishers
    ros::Publisher cmd_pub_fr_, cmd_pub_fl_, cmd_pub_rr_, cmd_pub_rl_, cmd_estop_;
    
    // Subscribers
    ros::Subscriber stat_callback_[NUM_MOTORS], fb_callback_[NUM_MOTORS],mcu_status_callback_;
    ros::Subscriber cmd_vel_sub_;
    // Timers
    ros::Timer encoder_watchdog_timer_, mcu_watchdog_timer_, mot_watchdog_timer_

      
    // A list serious faults, that would lear to immediate shutdown
    int serious_fault_[NUM_SERIOUS_FAULTS]

    // Diagnostic Packager
    MotionDiag motion_diag_(); 

    // Callback Functions
    void fr_stat_callback(const roboteq_msgs::Status);
    void fl_stat_callback(const roboteq_msgs::Status);
    void rr_stat_callback(const roboteq_msgs::Status);
    void rl_stat_callback(const roboteq_msgs::Status);

    void fr_fb_callback(const roboteq_msgs::Feedback);
    void fl_fb_callback(const roboteq_msgs::Feedback);
    void rr_fb_callback(const roboteq_msgs::Feedback);
    void rl_fb_callback(const roboteq_msgs::Feedback);

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

  serious_faults_[0] = roboteq_msgs::Status.FAULT_OVERHEAT;
  serious_faults_[1] = roboteq_msgs::Status.FAULT_OVERVOLTAGE;
  serious_faults_[2] = roboteq_msgs::Status.FAULT_SHORT_CIRCUIT;
  serious_faults_[3] = roboteq_msgs::Status.FAULT_MOSFET_FAILURE;

  mcu_heartbeat_rxd_ = false;
  mcu_dead_ = false;
  estop_status_ = grizzly_msgs::RawStatus.ERROR_ESTOP_RESET;
  precharge_malfunction_ = false
  pre_charge_timeout = 0;

  for (int i=0;i<NUM_MOTORS;i++) {
    mot_setting_[i] = 0;
    encreading_[i] = 0;

    enc_tics_in_error_[i] = 0; // track number of consecutive error measurements that were too large
    encoder_fault_[i] = false; // assume encoders are okay
    num_enc_fault_[i] = 0;
    mot_heartbeat_rxd_[i] = false;
    mot_node_dead_[i] = true;

    motor_fault_[i] = false;
  }
    
  stat_callback_[FR] = node_.subscribe("motors/front_right/status",1,&HardwareSafety::fr_stat_callback,this);
  stat_callback_[FL] = node_.subscribe("motors/front_right/status",1,&HardwareSafety::fr_stat_callback,this);
  stat_callback_[RR] = node_.subscribe("motors/front_right/status",1,&HardwareSafety::fr_stat_callback,this);
  stat_callback_[RL] = node_.subscribe("motors/front_right/status",1,&HardwareSafety::fr_stat_callback,this);

  fb_callback_[FR] = node_.subscribe("motors/front_right/feedback",1,&HardwareSafety::fr_fb_callback,this);
  fb_callback_[FL] = node_.subscribe("motors/front_left/feedback",1,&HardwareSafety::fr_fb_callback,this);
  fb_callback_[RR] = node_.subscribe("motors/rear_right/feedback",1,&HardwareSafety::fr_fb_callback,this);
  fb_callback_[RL] = node_.subscribe("motors/rear_left/feedback",1,&HardwareSafety::fr_fb_callback,this);

  mcu_status_callback = node_.subscribe("mcu/status",1,&HardwareSafety::mcu_stat_callback,this);

  encoder_watchdog_timer_ = node_.createTimer(ros::Duration(enc_watchdog_period_),&HardwareSafety::encoder_callback,this);
  mcu_watchdog_timer_ = node_.createTimer(ros::Duration(mcu_watchdog_time_), &HardwareSafety::mcu_watchdog_callback,this);
  mot_watchdog_timer_ = node_.createTimer(ros::Duration(mot_watchdog_time_), &HardwareSafety::mot_watchdog_callback,this);

  cmd_vel_sub_ = node_.subscribe("safe_cmd_vel",1,&HardwareSafety::vel_callback,this);
    
}

void HardwareSafety::mcu_watchdog_callback(const ros::TimerEvent& event) {
  if (mcu_heartbeat_rxd_) {
    mcu_dead_ = true;
    std::string out_str = "MCU Communication is inactive. Vehicle has been deactivated. Please reset systems";
    ROS_ERROR(out_str.c_str());
    //TODO:Publish diag
  }
  else {
    std::string out_str = "MCU Communication is active and OK";
    mcu_heartbeat_rxd = false;
    mcu_dead = false;
    //TODO:Publish diagnostics
  }

  if (estop_status_ == grizzly_msgs::RawStatus.ERROR_BRK_DET) { 
    pre_charge_timeout+=1;
  else
    pre_charge_timeout = 0;

  if (pre_charge_timeout > (4.0/(mcu_watchdog_time_))) {
    precharge_malfunction_ = true;
    std_msgs::Bool estop_out;
    estop_out.data = true;
    cmd_estop_.publish(estop_out);
    std::string error_str = "Precharge malfunction. EStop activated. Please reboot all systems";
    ROS_ERROR(error_str.c_str());
    //TODO:Publish diagnostics
  }
  else if (estop_status_!=0) {
    std::string warn_str = "EStop is activate. Vehicle will not move";
    precharge_malfunction_ = false;
    ROS_WARN(warn_str);
    //TODO:Publish diagnostics
  }
  else {
    precharge_malfunction_ = false;
    std::string ok_str = "EStop System Open and Functional";
    //TODO:Publish diagnostics
  }


}

void HardwareSafety::mcu_stat_callback(grizzly_msgs::RawStatus& data) {
  mcu_heartbeat_rxd_ = true;
  estop_status_ = data->error;
}

void HardwareSafety::vel_callback(const geometry_msgs::Twist& data)
{
  double right_speed = data->linear.x + (data->angular.z)*(width_/2.0);
  double left_speed = data->linear.x - (data->angular.z)*(width_/2.0);

  mot_setting_[FR] = -right_speed * roboteq_scale_;
  mot_setting_[FL] = left_speed * roboteq_scale_;
  mot_setting_[RR] = -right_speed * roboteq_scale_;
  mot_setting_[RL] = left_speed * roboteq_scale_;

  //Dont send command if
  //a) Motor is fault (incl. encoder issues)
  //b) Motor node is dead
  //c) MCU node is dead
  //d) Estop is not cleared and/or pre-charge is not completed
  //e) There is an encoder fault

  bool fault_check = true; //assume everything is okay. 
  for (int i = 0;i<NUM_MOTORS;i++) {
    //ideally all faults should be false
    fault_check = !motor_fault_[i];
    fault_check = !mot_node_dead_[i];
    fault_check = !encoder_fault_[i];
  }

  if (mcu_dead_) 
    fault_check = false;  

  if (estop_status_!=0)
    fault_check = false;

  roboteq_msgs::Command data_out;
  if (fault_check) {
    data_out.setpoint = mot_setting_[FR];
    cmd_pub_fr.publish(data_out);
    data_out.setpoint = mot_setting_[FL];
    cmd_pub_fl.publish(data_out);
    data_out.setpoint = mot_setting_[RR];
    cmd_pub_rr.publish(data_out);
    data_out.setpoint = mot_setting_[RL];
    cmd_pub_rl.publish(data_out);
  }
  else {
    data_out.setpoint = 0;
    cmd_pub_fr.publish(data_out);
    cmd_pub_fl.publish(data_out);
    cmd_pub_rr.publish(data_out);
    cmd_pub_rl.publish(data_out);
  }
} 



void HardwareSafety::fr_stat_callback(const roboteq_msgs::Status& data) {
  check_motor(data->fault, FR);
  mot_heartbeat_rxd_[FR] = true;
}

void HardwareSafety::fl_stat_callback(const roboteq_msgs::Status& data) {
  check_motor(data->fault, FL);
  mot_heartbeat_rxd_[FL] = true;
}

void HardwareSafety::rr_stat_callback(const roboteq_msgs::Status& data)
 {
   check_motor(data->fault,RR);
   mot_heartbeat_rxd_[RR] = true;
}
void HardwareSafety::rl_stat_callback(const roboteq_msgs::Status& data) {
  check_motor(data->fault,RL);
  mot_heartbeat_rxd_[RL] = true;
}

void mot_watchdog_callback(const ros::TimerEvent& event) {
  for (int i=0;i<NUM_MOTORS;i++) {
    

  }


}

void HardwareSafety::fr_fb_callback(const roboteq_msgs::Feedback& data) {

}

void HardwareSafety::fl_fb_callback(const roboteq_msgs::Feedback& data) {

}

void HardwareSafety::rr_fb_callback(const roboteq_msgs::Feedback& data) {

}

void HardwareSafety::rl_fb_callback(const roboteq_msgs::Feedback& data) {

}





class MotionDiag {
  public:
    MotionDiag {


    }
  private:
    

};
















  

}
