#include <ros.h>
#include <ros/time.h>
#include <mitro_base_controller/VelocityCommand.h>
#include <mitro_base_controller/JointStates.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ArduinoHardware.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <RunningAverage.h>

RunningAverage avg_speed_r(20);
RunningAverage avg_speed_l(20);

#include <Servo.h>

//#define ENCODER_OPTIMIZE_INTERRUPTS // conflicts with attachInterrupt()
#include <Encoder.h>

Encoder myEncL(18, 20);
Encoder myEncR(21, 19); // reversed for sign

# define Relais              2              
# define EmergencyDetect     3
# define VoltageReader       A2

//# define MAX_SPEED     30.0

std_msgs::Float32 battery_msg;
std_msgs::Bool runstop_msg;
std_msgs::Bool relais_msg;
ros::Publisher pub_battery("battery_voltage", &battery_msg);
ros::Publisher pub_runstop("runstop", &runstop_msg);
ros::Publisher pub_relais("relais", &relais_msg);
mitro_base_controller::JointStates js_msg;
ros::Publisher js_pub("joint_states",&js_msg);
ros::NodeHandle nh;

Servo M1Control;
Servo M2Control;

double M1previous_error = 0;
double M1integral = 0;
long M1Prevtime = millis()-1;
double M1ActualSpeed = 0;
double M1SpeedSetpoint = 0;
double M1PrevSetpoint = 0;
double M1PulseLength = 1500;
double M2previous_error = 0;
double M2integral = 0;
long M2Prevtime = millis()-1;
double M2ActualSpeed = 0;
double M2SpeedSetpoint = 0;
double M2PrevSetpoint = 0;
double M2PulseLength = 1500;

double loop_delay = 10; // 100 Hz
double js_delay = 50; // 20 Hz
double status_delay = 1000; // 1Hz

long prev_time = 0;

boolean relais = false;

long PositionL  = 0;
long PositionR  = 0;
long   prevPosL = 0;
long   prevPosR = 0;


void vc_cb( const mitro_base_controller::VelocityCommand& cmd_vel_msg) {
  M1PrevSetpoint = M1SpeedSetpoint;
  M2PrevSetpoint = M2SpeedSetpoint;
  M1SpeedSetpoint = cmd_vel_msg.velocity_right;//min(max(cmd_vel_msg.velocity_right, - MAX_SPEED), MAX_SPEED);
  M2SpeedSetpoint = cmd_vel_msg.velocity_left;//min(max(cmd_vel_msg.velocity_left, - MAX_SPEED), MAX_SPEED);
  if ((M1SpeedSetpoint * M1PrevSetpoint < 0) || (M1SpeedSetpoint == 0)) {
    M1integral = 0.0;
  }
  if ((M2SpeedSetpoint * M2PrevSetpoint < 0) || (M2SpeedSetpoint == 0)) {
    M2integral = 0.0;
  }
}


void cmd_rel_cb( const std_msgs::Bool& cmd_rel_msg) {
  if (cmd_rel_msg.data == true) {
    digitalWrite(Relais, LOW);
    relais_msg.data = true;
    relais = true;
  }
  else {
    digitalWrite(Relais, HIGH);
    relais_msg.data = false;
    relais = false;
  }
}

ros::Subscriber<mitro_base_controller::VelocityCommand> sub_cmd_vel("cmd_vel", vc_cb);
ros::Subscriber<std_msgs::Bool> sub_relais("cmd_relais", cmd_rel_cb);


// Since we are hooking into a standard arduino sketch, we must define our program in
// terms of the arduino setup and loop functions.

void setup(){

  // setup motors & relais
  pinMode(Relais, OUTPUT);
  digitalWrite(Relais, HIGH);

  pinMode(EmergencyDetect, INPUT);

  M1Control.attach(9);
  M2Control.attach(10);

  nh.getHardware()->setBaud(115200);
  
  avg_speed_r.clr();
  avg_speed_l.clr();

  // ROS
  nh.initNode();
  nh.advertise(js_pub);
  nh.advertise(pub_battery);
  nh.advertise(pub_runstop);
  nh.advertise(pub_relais);
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_relais);
  js_msg.position_right = 0;
  js_msg.position_left = 0;
  js_msg.velocity_right = 0;
  js_msg.velocity_left = 0;
  relais_msg.data = false;
}


long js_timer;
long status_timer;
long current_time = millis() - loop_delay;

void loop() {

  prev_time = current_time;
  current_time = millis();

  Query_Speed();

  boolean runstop = !digitalRead(EmergencyDetect);

  /* 
   M1SpeedSetpoint = 1600;
   M2SpeedSetpoint = 0;
   
   if (current_time - temp_timer > 10000) {
   M1SpeedSetpoint = 3200;
   }
   if (current_time - temp_timer > 20000) {
   M1SpeedSetpoint = 0;
   }
   if (current_time - temp_timer > 30000) {
   M1SpeedSetpoint = -3200;
   }
   */

  if (runstop || !relais) { // || millis() - last_vel_cmd_time > 200 ) {
    M1Control.write(1500);
    M2Control.write(1500);
    M1PulseLength = 1500.0;
    M2PulseLength = 1500.0;
    M1SpeedSetpoint = 0.0;
    M2SpeedSetpoint = 0.0;
    M1integral = 0.0;
    M2integral = 0.0;
    avg_speed_r.clr();
    avg_speed_l.clr();
  } 
  else {
    //M1PulseLength = 1500 + M1SpeedSetpoint * (500.0 / 6400.0);
    M1PulseLength = 1500;
    M1PulseLength += M1PID(M1SpeedSetpoint, M1ActualSpeed, 7.5, 3.75, 0.0);
    M1PulseLength = min(max(M1PulseLength, 1050), 1950);
    if (M1SpeedSetpoint == 0) {// && abs(M1PulseLength - 1500) < 100) {
      M1PulseLength = 1500;
    }
    M1Control.write((int) M1PulseLength);
 
    //M2 = left
    //M2PulseLength = 1500 + M2SpeedSetpoint * (500.0 / 6400.0);
    M2PulseLength = 1500;
    M2PulseLength += M2PID(M2SpeedSetpoint, M2ActualSpeed, 7.5, 3.75, 0.0);
    M2PulseLength = min(max(M2PulseLength, 1050), 1950);
    if (M2SpeedSetpoint == 0) {// && abs(M2PulseLength - 1500) < 100) {
      M2PulseLength = 1500;
    }
    M2Control.write((int) M2PulseLength);
  }
  
  if (millis() > js_timer) {
    js_timer = millis() + js_delay;
    js_msg.header.stamp = nh.now();
    js_msg.position_right = PositionR; //M1PulseLength;
    js_msg.position_left = PositionL; // M2PulseLength;
    js_msg.velocity_right = M1ActualSpeed;
    js_msg.velocity_left = M2ActualSpeed;
    js_pub.publish(&js_msg);
  }


  if (millis() > status_timer) {
    status_timer = millis() + status_delay;
    int voltage_reading = analogRead(VoltageReader);
    battery_msg.data = voltage_reading / 1024.0 * 20.95;
    runstop_msg.data = runstop;
    pub_runstop.publish(&runstop_msg);
    pub_battery.publish(&battery_msg);
    pub_relais.publish(&relais_msg);
  }

  nh.spinOnce();
  long dt = millis() - current_time;
  delay(max(0, loop_delay - dt));
}


double M1PID(double setpoint, double actual, double Kp, double Ki, double Kd) {

  double error = (double)(setpoint - actual) / 100.0;

  long time = millis();
  double dt = (double)(time - M1Prevtime)/1000.0;
  M1Prevtime = time;

  M1integral += (error*dt);

  double derivative = (error - M1previous_error)/dt;
  double output = (Kp*error) + (Ki*M1integral) + (Kd*derivative);
  M1previous_error = error;

  return output;
}

double M2PID(double setpoint, double actual, double Kp, double Ki, double Kd) {

  double error = (double)(setpoint - actual) / 100.0;

  long time = millis();
  double dt = (double)(time - M2Prevtime)/1000.0;
  M2Prevtime = time;

  M2integral += (error*dt);

  double derivative = (error - M2previous_error)/dt;
  double output = (Kp*error) + (Ki*M2integral) + (Kd*derivative);
  M2previous_error = error;

  return output;
}

void Query_Speed(){
  double dt = (double)(current_time - prev_time); // should be around 10 for 100Hz

  PositionL = myEncL.read();
  PositionR = myEncR.read();

  //avg_speed_r = avg_speed_r * 0.9 + ((PositionR - prevPosR) / (dt / 1000.0)) * 0.1;
  //avg_speed_l = avg_speed_l * 0.9 + ((PositionL - prevPosL) / (dt / 1000.0)) * 0.1;
  
  avg_speed_r.add((PositionR - prevPosR) / (dt / 1000.0));
  avg_speed_l.add((PositionL - prevPosL) / (dt / 1000.0));
  
  M1ActualSpeed = avg_speed_r.avg(); //(PositionR - prevPosR) / (dt / 1000.0);
  M2ActualSpeed = avg_speed_l.avg(); //(PositionL - prevPosL) / (dt / 1000.0);

  prevPosL = PositionL;
  prevPosR = PositionR;
}
