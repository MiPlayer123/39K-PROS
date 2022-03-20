#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

// CONTROLLER
Controller controller(ControllerId::master); 

// MOTORS
Motor LeftRear(14, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor LeftMid(6, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor LeftFront(13, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor RightRear(20, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor RightMid(17, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor RightFront(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::rotations);
Motor Bar(1, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor Intake(18, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
MotorGroup leftDrive({LeftRear, LeftMid, LeftFront});
MotorGroup rightDrive({RightRear, RightMid, RightFront});

// SENSORS
IMU Intertial(19);
IMU ClawInertial(12, IMUAxes::y);
RotationSensor LOdom(9);
RotationSensor ROdom(16, true);
RotationSensor BarRot(2);
ADIButton FrontSense('C');

// PNEUMATICS
pros::ADIDigitalOut Pullback('F');
pros::ADIDigitalOut RearClaw('E');
pros::ADIDigitalOut FrontClaw('D'); 
pros::ADIDigitalOut Pn('H');

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftDrive, rightDrive)
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    .withDimensions({AbstractMotor::gearset::blue, 3/7}, {{4_in, 13.5_in}, imev5BlueTPR})
    .withSensors(LOdom, ROdom)
    .withOdometry({{2.75_in, 8.25_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
    .buildOdometry();

std::shared_ptr<AsyncMotionProfileController> profileController =
AsyncMotionProfileControllerBuilder()
    .withLimits({
        10.0, // Maximum linear velocity of the Chassis in m/s
        10.0, // Maximum linear acceleration of the Chassis in m/s/s
        15.0 // Maximum linear jerk of the Chassis in m/s/s/s
})
.withOutput(chassis)
.buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> barControl =
  AsyncPosControllerBuilder()
  .withMotor(Bar)
  .withGains({0.007, 0.0, 0.000075})
  .build();

void setBrakes(){
    /*
    leftDrive.setBrakeMode(brake);
    rightDrive.setBrakeMode(brake);
    Bar.setBrakeMode(hold);
    Intake.setBrakeMode(coast);
    */
}