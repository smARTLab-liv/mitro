#include <ros.h>
#include <mitro_sonar/SonarRaw.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <ArduinoHardware.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

const int pingPin1 = 3; // far left
const int pingPin2 = 4; // left center
const int pingPin3 = 5; // center
const int pingPin4 = 6; // right center
const int pingPin5 = 7; // far right
const int relaisCtl = 8; // relais control
const int runstopIn = 9; // runstop input

const long interval = 200;
const long msg_interval = 1000;

boolean relais = false;

void cmd_rel_cb( const std_msgs::Bool& cmd_rel_msg) {
  if (cmd_rel_msg.data == true) {
    set_relais(true);
  }
  else {
    set_relais(false);
  }
}

mitro_sonar::SonarRaw sonar_msg;
std_msgs::Bool runstop_msg;
std_msgs::Bool relais_msg;
ros::Publisher sonar_pub("sonar_raw", &sonar_msg);
ros::Publisher pub_runstop("runstop", &runstop_msg);
ros::Publisher pub_relais("relais", &relais_msg);
ros::Subscriber<std_msgs::Bool> sub_relais("cmd_relais", cmd_rel_cb);
ros::NodeHandle nh;

void set_relais(boolean set) {
  if (set) {
    digitalWrite(relaisCtl, HIGH);
    relais_msg.data = true;
    relais = true;
  }
  else {
    digitalWrite(relaisCtl, LOW);
    relais_msg.data = false;
    relais = false;
  }
}

void setup() {
  pinMode(relaisCtl, OUTPUT);
  digitalWrite(relaisCtl, LOW);

  pinMode(runstopIn, INPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sonar_pub);
  nh.advertise(pub_relais);
  nh.advertise(pub_runstop);
  nh.subscribe(sub_relais);
  relais_msg.data = false;
}

long prev_time = millis();
long prev_msg = millis();

void loop()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, del, cm;
  int pingPin;

  for (int i = 1; i <= 5; i++) {
    if (i == 1) {
      pingPin = pingPin1;
    }
    else if (i == 2) {
      pingPin = pingPin2;
    }
    else if (i == 3) {
      pingPin = pingPin3;
    }
    else if (i == 4) {
      pingPin = pingPin4;
    }
    else if (i == 5) {
      pingPin = pingPin5;
    }

    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);

    // convert the time into a distance
    //inches = microsecondsToInches(duration);
    cm = microsecondsToCentimeters(duration);

    if (i == 1) {
      sonar_msg.s1_dist = cm;
    }
    else if (i == 2) {
      sonar_msg.s2_dist = cm;
    }
    else if (i == 3) {
      sonar_msg.s3_dist = cm;
    }
    else if (i == 4) {
      sonar_msg.s4_dist = cm;
    }
    else if (i == 5) {
      sonar_msg.s5_dist = cm;
    }
  }

  sonar_pub.publish(&sonar_msg);
  
  boolean runstop = !digitalRead(runstopIn);
  set_relais(runstop);
  
  if (millis() - prev_msg > msg_interval - interval) {
    runstop_msg.data = runstop;
    pub_relais.publish(&relais_msg);
    pub_runstop.publish(&runstop_msg);
  }
  
  nh.spinOnce();

  del = millis() - prev_time;
  prev_time = millis();
  delay(max(0, interval - del));
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}



