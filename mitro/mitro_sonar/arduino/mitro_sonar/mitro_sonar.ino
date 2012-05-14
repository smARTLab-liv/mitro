#include <ros.h>
#include <mitro_sonar/SonarRaw.h>

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

const long interval = 200;

mitro_sonar::SonarRaw sonar_msg;
ros::Publisher sonar_pub("sonar_raw", &sonar_msg);
ros::NodeHandle nh;

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sonar_pub);
}

long prev_time = millis();

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



