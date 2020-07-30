/*
 * File: bluerov/src/simple_pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends actuator commands to a mavlink controller.
 */

#include <vector>
#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/CommandLongRequest.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov/simple_pilotConfig.h>

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    // ros::Publisher mavlink_pub;
    ros::ServiceClient command_client;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber hazard_enable_sub;
    ros::Publisher rc_override_pub;

    ros::Subscriber cmd_mode_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient mode_client;

    dynamic_reconfigure::Server<bluerov::simple_pilotConfig> server;
    bluerov::simple_pilotConfig config;

    bool hazards_enabled;
    

    void configCallback(bluerov::simple_pilotConfig &update, uint32_t level);
    void setServo(int index, float pulse_width);//parameter is not a pulse width
    void overrideRC(float* rc_in);
    void setArmed(bool armed);
    void setMode(uint8_t mode);
    void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    void hazardCallback(const std_msgs::Bool::ConstPtr& msg);
    void modeCallback(const std_msgs::UInt8::ConstPtr& cmd_mode);
};

Pilot::Pilot() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov::simple_pilotConfig>::CallbackType f;
  f = boost::bind(&Pilot::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  cmd_mode_sub = nh.subscribe<std_msgs::UInt8>("cmd_mode", 1, &Pilot::modeCallback, this);
  cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pilot::velCallback, this);
  hazard_enable_sub = nh.subscribe<std_msgs::Bool>("hazard_enable", 1, &Pilot::hazardCallback, this);

  // set initial values
  hazards_enabled = false;


  rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1);
}

void Pilot::spin() {
  // enforce a max spin rate so we don't kill the CPU
  ros::Rate loop(1000); // Hz

  while(ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();
    loop.sleep();
  }
}

void Pilot::configCallback(bluerov::simple_pilotConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

void Pilot::setServo(int index, float value) {
  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  int pulse_width = (value + 1) * 400 + 1100;

  // send mavros command message
  // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
  // CMD_DO_SET_SERVO (183): https://pixhawk.ethz.ch/mavlink/
  mavros_msgs::CommandLong srv;
  //srv.request.command = mavros::CommandLongRequest::CMD_DO_SET_SERVO;
  srv.request.command = 183u;
  srv.request.param1 = index + 1; // servos are 1-indexed here
  srv.request.param2 = pulse_width;
  bool result = command_client.call(srv);
  //ROS_INFO_STREAM("Pilot::setServo(" << index << ", " << value << ") = " << result);
}

void Pilot::overrideRC(float values[8]) {
  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  uint16_t pulse_width[8];
  for(int i = 0; i < 8; i++) {
    pulse_width[i] = (values[i] + 1) * 500 + 1000;//using range 1000-2000 for arming/disarming purposes
//if we always output 1000-2000 on pi, actual ranges to escs can be configured/limited via apm parameters
  }

  mavros_msgs::OverrideRCIn rc;
  //rc.request.command = 70u;
  //rc.request.param1 = index + 1; // servos are 1-indexed here
  for(int i = 0; i < 8; i++) {
    rc.channels[i] = pulse_width[i];
  }
  rc.channels[4] = 65535;
  rc_override_pub.publish(rc);
}

void Pilot::setArmed(bool armed) {
  mavros_msgs::CommandBool arm;
  arm.request.value = armed;
  bool result = arming_client.call(arm);

}

void Pilot::setMode(uint8_t mode) {

  mavros_msgs::OverrideRCIn rc;
  for(int i = 0; i < 8; i++) {
	  rc.channels[i] = 65535; // No change flag
  }

  switch(mode) {

  case 0:
		  rc.channels[4] = 1000;
  	  	  break;
  case 2:
		  rc.channels[4] = 2000;
  	  	  break;
  case 9:
	  	  rc.channels[4] = 1500;
	  	  break;
  default:
	  	  ROS_INFO("mode failed");
	  	  break;

  }
  rc_override_pub.publish(rc);
//  mavros_msgs::SetMode set;

//  set.request.base_mode = 0;
//  set.request.custom_mode = "3";
//  bool result = mode_client.call(set);
}

void Pilot::modeCallback(const std_msgs::UInt8::ConstPtr& cmd_mode) {
  setMode(cmd_mode->data);

}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  // only continue if hazards are enabled
  if(!hazards_enabled) return;

  // extract cmd_vel message
  float roll     = cmd_vel->angular.y;
  float strafe    = cmd_vel->angular.x;
  float pitch      = cmd_vel->angular.z;
  float forward  = cmd_vel->linear.x;
  float yaw   = cmd_vel->linear.y;
  float vertical = cmd_vel->linear.z;
/*
  // build thruster commands (expected to be between -1 and 1)
  float thruster[6];
  thruster[0] =
    roll +
    -config.front_forward_decouple * forward +
    -config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control; // Vertical Left (VL)
  thruster[1] =
    config.front_forward_decouple * forward +
    pitch +
    vertical +
    config.buoyancy_control; // Vertical Back (VB)
  thruster[2] =
    -roll +
    -config.front_forward_decouple * forward +
    config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control;  // Vertical Right (VR)
  thruster[3] = -yaw + forward; // Forward Left (FL)
  thruster[4] = strafe; // LATeral (LAT)
  thruster[5] = yaw + forward; // Forward Right (FR)
*/

/*
  float rc_in[8];
	rc_in[0] = pitch;
	rc_in[1] = roll;//yaw
	rc_in[2] = vertical;
	rc_in[3] = yaw;//strafe
	rc_in[4] = -1.0f;
	rc_in[5] = -1.0f;
	rc_in[6] = forward;
	rc_in[7] = strafe;//roll

*/

//Remap channels
//Right joystick: u/d = pitch, l/r = roll
//Left joystick: u/d = thrust(throttle forward), l/r = yaw
//Right/Left triggers: climb/descend
//Right/Left bumpers: strafe left/right
//ToDo: fix up names, maybe use channel mapper
  float rc_in[8];
	rc_in[0] = -pitch;
	rc_in[1] = -roll;//yaw
	rc_in[2] = vertical;
	rc_in[3] = -yaw;//strafe
	rc_in[4] = -1.0f;
	//rc_in[5] = -1.0f;
	//rc_in[5] = mode_rc;
	rc_in[5] = forward;
	rc_in[6] = strafe;//roll
	rc_in[7] = -1.0f;


/*
  // send thruster positions
  for(int i = 0; i < 6; i++) {
   setServo(i, thruster[i]);
  }
*/
  // send rc positions

  overrideRC(rc_in);

}

void Pilot::hazardCallback(const std_msgs::Bool::ConstPtr& msg) {
  // save message data
  hazards_enabled = msg->data;
  if(hazards_enabled) {
    setArmed(true);
    ROS_INFO("Enabled thrusters.");

  } else ROS_INFO("Disabled thrusters.");

  // zero thruster speeds
  if(!hazards_enabled) {
    for(int i = 0; i < 6; i++) {
     setArmed(false);
     setServo(i, 0);
	//ToDo: send mavros message to disarm, set overrideRC to go into failsafe mode on apm 
//overrideRC(rc_zero)
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
