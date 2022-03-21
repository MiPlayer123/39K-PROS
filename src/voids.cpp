#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

void spinIntake(){
  Intake.moveVoltage(12000);
}

void stopIntake(){
  Intake.moveVoltage(0);
}

void openClaw(){
  FrontClaw.set_value(false);
}

void closeClaw(){
  FrontClaw.set_value(true);
}

void grabRear(){
  RearClaw.set_value(true);
  if(ClawInertial.get()<50 && ClawInertial.get()>-20){
    Pullback.set_value(true);
  }
}

void releaseRear(){
  Pullback.set_value(false);
  RearClaw.set_value(false);
}