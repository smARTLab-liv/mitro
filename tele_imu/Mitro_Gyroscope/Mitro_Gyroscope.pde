#include <TimerOne.h>

const int pinGyroX = A0;  // Analog input pin that the potentiometer is attached to
const int pinGyroZ = A1;
const int pinAccX = A2;
const int pinAccY = A3;
const int pinAccZ = A4;

int gyroX = 0; 
int gyroZ = 0;
int accX =0;
int accY = 0;
int accZ = 0;
int sample = 0;
boolean newMeasurement = false;

void setup() {
  analogReference(EXTERNAL); // put 3.3v reference supply on there
  pinMode(13, OUTPUT);
  Serial.begin(1000000); // fast serial speed for minimum overhead
  Timer1.initialize(1000);         // initialize timer interupt
  Timer1.attachInterrupt(callback);  

}

void callback()
{
  gyroX += analogRead(pinGyroX);    
  gyroZ += analogRead(pinGyroZ); 
  accX += analogRead(pinAccX);
  accY += analogRead(pinAccY);
  accZ += analogRead(pinAccZ);
  sample ++;
  newMeasurement = true;
}


void loop() {

  if (newMeasurement==true)
  {
    cli(); //disable interrupts during data copy
    int tempX = gyroX; 
    gyroX = 0;
    int tempZ = gyroZ;
    gyroZ = 0;
    int tempCnt = sample;
    sample = 0;
    int tempAccX = accX;
    accX = 0;
    int tempAccY = accY;
    accY = 0;
    int tempAccZ = accZ;
    accZ = 0;
    newMeasurement = false;
   sei(); // enable interrupts
                    

    Serial.write(highByte(tempX));    
    Serial.write(lowByte(tempX));         
    Serial.write(highByte(tempZ));  
    Serial.write(lowByte(tempZ));     
    Serial.write(highByte(tempAccX));  
    Serial.write(lowByte(tempAccX));     
    Serial.write(highByte(tempAccY));  
    Serial.write(lowByte(tempAccY));     
    Serial.write(highByte(tempAccZ));  
    Serial.write(lowByte(tempAccZ));       
    Serial.write(tempCnt);  
    Serial.write(10);

  }
}






