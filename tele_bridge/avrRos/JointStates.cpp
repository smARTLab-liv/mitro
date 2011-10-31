#include "JointStates.h"
#include <stdio.h>
using namespace tele_msgs ;

JointStates::JointStates() {} 


 JointStates::~JointStates(){
}

 JointStates::JointStates(uint8_t* data){
}

uint16_t  JointStates::serialize(uint8_t* data){
int offset=0;
*( (int32_t *) (data + offset))=  this->position_right; 
offset += 4;
*( (int32_t *) (data + offset))=  this->position_left; 
offset += 4;
*( (float *) (data + offset))=  this->velocity_right; 
offset += 4;
*( (float *) (data + offset))=  this->velocity_left; 
offset += 4;

 return offset;

}

uint16_t  JointStates::deserialize(uint8_t* data){
int offset=0;
this->position_right = *( (int32_t *) (data + offset) );
offset += 4;
this->position_left = *( (int32_t *) (data + offset) );
offset += 4;
this->velocity_right = *( (float *) (data + offset) );
offset += 4;
this->velocity_left = *( (float *) (data + offset) );
offset += 4;

 return offset;

}

uint16_t  JointStates::bytes(){
 int msgSize=0;
msgSize += sizeof(int32_t);
msgSize += sizeof(int32_t);
msgSize += sizeof(float);
msgSize += sizeof(float);
return msgSize;

}
