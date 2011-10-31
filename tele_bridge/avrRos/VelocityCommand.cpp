#include "VelocityCommand.h"
#include <stdio.h>
using namespace tele_msgs ;

VelocityCommand::VelocityCommand() {
} 


VelocityCommand::~VelocityCommand(){
}

VelocityCommand::VelocityCommand(uint8_t* data){
}

uint16_t  VelocityCommand::serialize(uint8_t* data){
  int offset=0;
  *( (float *) (data + offset))=  this->velocity_right; 
  offset += 4;
  *( (float *) (data + offset))=  this->velocity_left; 
  offset += 4;

  return offset;

}

uint16_t  VelocityCommand::deserialize(uint8_t* data){
  int offset=0;
  this->velocity_right = *( (float *) (data + offset) );
  offset += 4;
  this->velocity_left = *( (float *) (data + offset) );
  offset += 4;

  return offset;

}

uint16_t  VelocityCommand::bytes(){
  int msgSize=0;
  msgSize += sizeof(float);
  msgSize += sizeof(float);
  return msgSize;

}

