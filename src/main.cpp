#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
//using namespace autolib;
 
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

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
    //Odom display
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
	display.setStateCallback(setStateOdom);
	display.setResetCallback(resetSensors);
    auto state = chassis->getState();
    //display.setData({state.y, state.x, state.theta},{LOdom.get(), ROdom.get()});

    //inits
    Bar.tarePosition();
    barControl->reset();
    setBrakes();
    /*
    barControl->setTarget(200);
    chassis->setState({0_in, 0_in, 0_deg});
    chassis->driveToPoint({1_ft, 0_ft});
    barControl->waitUntilSettled();
    display.setData({state.y, state.x, state.theta},{LOdom.get(), ROdom.get()});
    */
    chassis->setState({0_ft, 0_ft, 0_deg});
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
    //barControl->setTarget(200);
    //barControl->setTarget(150);
    autolib::PathGenerator pathGenerator({1.0, 2.0, 4.0});
    //barControl->setTarget(150);
    pathGenerator.generatePath({ autolib::Pose{ 1_ft, 1_ft, 0_deg }, autolib::Pose{ 2_ft, 2_ft, 270_deg }, autolib::Pose{ 2_ft, 3_ft, 270_deg }}, 
    std::string("test")
    );
    //barControl->setTarget(100);
    autolib::PurePursuit purePursuit( pathGenerator.getPaths(), 1_in );
    barControl->setTarget(200);
    barControl->waitUntilSettled();
    //autolib::PurePursuitTriangle triangle = purePursuit.run( state, std::string("test") );
    purePursuit.updateChassis( 50, purePursuit.run( state, std::string("test") ), chassis );
    barControl->setTarget(100);
    barControl->waitUntilSettled();
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

    leftDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
    while(true) {
        chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));

        auto state = chassis->getState();
        display.setData({state.y, state.x, state.theta},{LOdom.get(), ROdom.get()});

        pros::delay(10);
    }
}