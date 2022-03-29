#pragma once
#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

// CONTROLLER
extern Controller controller;

// MOTORS
extern MotorGroup leftDrive;
extern MotorGroup rightDrive;
extern Motor Bar;
extern Motor Intake;

// SENSORS
extern IMU Inertial;
extern RotationSensor LOdom;
extern RotationSensor ROdom;
extern ADIButton FrontSense;

// PNEUMATICS
extern pros::ADIDigitalOut RearClaw;
extern pros::ADIDigitalOut FrontClaw; 
extern pros::ADIDigitalOut Pn;

//SYSTEM CONTROLLERS
extern std::shared_ptr<OdomChassisController> chassis;
namespace autolib{extern std::shared_ptr<OdomChassisController> auto_chassis;}
extern std::shared_ptr<AsyncMotionProfileController> profileController;
extern std::shared_ptr<AsyncPositionController<double, double>> barControl;

void setBrakes();