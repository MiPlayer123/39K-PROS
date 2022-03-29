#include "main.h"
#include "okapi/api.hpp"
#include "okapi/pathfinder/include/pathfinder.h"
using namespace okapi;

void setStateOdom(OdomDebug::state_t state) {
	chassis->setState({state.x, state.y, state.theta});
}
void resetSensors() {
	// reset sensors and reset odometry
	chassis->setState({0_in, 0_in, 0_deg});
    ROdom.reset();
    LOdom.reset();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    Inertial.reset(); //Change to calibrate later
    ROdom.reset();
    LOdom.reset();
    initialize_kalman();
    /*
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    pros::lcd::register_btn1_cb(on_center_button);
     */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}
 
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
 
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() { 
    //inits
    Bar.tarePosition();
    barControl->reset();
    setBrakes();
    chassis->setState({0_ft, 0_ft, 0_deg});

    /*
    barControl->setTarget(200);
    chassis->setState({0_in, 0_in, 0_deg});
    chassis->driveToPoint({1_ft, 0_ft});
    barControl->waitUntilSettled();
    display.setData({state.y, state.x, state.theta},{LOdom.get(), ROdom.get()});
    */
    //chassis->moveDistance(1_ft);
    //chassis->turnAngle(90_deg);
    /*
    barControl->setTarget(200);
	profileController->generatePath({
        {0_ft, 0_ft, 0_deg}, 
        {2_ft, 2_ft, 270_deg},
        {3_ft, 3_ft, 270_deg}}, 
        "A"
    );
	profileController->setTarget("A");
	profileController->waitUntilSettled();
    //chassis->driveToPoint({3_ft, 1_ft});
    barControl->waitUntilSettled();
    closeClaw();
    */
    
    autolib::PathGenerator pathGenerator({1.0, 2.0, 4.0});
    pathGenerator.generatePath({ autolib::Pose{ 1_ft, 1_ft, 0_deg }, autolib::Pose{ 2_ft, 2_ft, 270_deg }}, 
    std::string("test")
    );
    autolib::PurePursuit purePursuit( pathGenerator.getPaths(), 1_ft );
    auto auto_state = autolib::auto_chassis->getState();
    //barControl->setTarget(100);
    //autolib::PurePursuitTriangle triangle = purePursuit.run( auto_state, std::string("test") );
    //barControl->setTarget(150);
    purePursuit.updateChassis( 50, purePursuit.run( auto_state, std::string("test")), autolib::auto_chassis );
    //barControl->setTarget(100);
    //barControl->waitUntilSettled();
}
 
 
 
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 
 
 
void opcontrol() {
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
	display.setStateCallback(setStateOdom);
	display.setResetCallback(resetSensors);

    //leftDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
    //rightDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
    while(true) {
        chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));

        auto state = chassis->getState();
        display.setData({state.y, state.x, state.theta},{LOdom.get(), ROdom.get()});

        pros::delay(10);
    }
}