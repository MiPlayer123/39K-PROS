#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

//For PID turns
#define TURN_KP 0.05
#define TURN_KI 0.0018
#define TURN_KD 0.001
#define TURN_MAX_A (BASE_MAX_V / 0.1)
#define TURN_MAX_V (BASE_MAX_V * 0.7)
#define TURN_MIN_V 3

//For heavy PID turns 
#define TURN_KP_HEAVY 0.035
#define TURN_KI_HEAVY 0.005 //0.0048
#define TURN_KD_HEAVY 0.001

//For main inertial_drive
#define   kp 4 //8/7 
#define   ki .0008 //.5
#define   kd 1 //.45
#define integral_threshold 10
#define kp_c .45 //.42

//For PD balance
#define   kp_bal 1 
#define   kd_bal .5 

///* Voids *///
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
  /*
  if(ClawInertial.get()<50 && ClawInertial.get()>-20){
    Pullback.set_value(true);
  }
  */
  pros::delay(250);
  Pullback.set_value(true);
}

void releaseRear(){
  Pullback.set_value(false);
  RearClaw.set_value(false);
}

void startBar(int volt){
  Bar.moveVoltage(volt*1000);
}

void stopBar(){
  Bar.moveVoltage(0);
}

///* PIDs *///
// Turn to an absolute rotation
void turn_absolute_inertial(double target, bool heavy) {
  double last_error = 1000;
  double last_output = 0;
  double integral = 0;

  //long ticks = 0;
  long brake_cycles = 0;

  while (true) {
    // Calculate the error
    double error = target - get_rotation();

    // Initialize the last error if it was not already
    if (ae(last_error, 1000)) {
      last_error = error;
    }

    // Calculate the integral and derivative if we're within the bound
    double derivative;
    if (fabs(error) > 10) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    // Calculate the raw output and update the last erro
    double raw_output;
    if(heavy){
      raw_output = TURNING_RADIUS * (TURN_KP_HEAVY * error + TURN_KI_HEAVY * integral + TURN_KD_HEAVY * derivative);
    }else{
      raw_output = TURNING_RADIUS * (TURN_KP * error + TURN_KI * integral + TURN_KD * derivative); // in/s
    }
    last_error = error;

    // Constrain acceleration between iterations and calculate the output
    double acceleration = clamp((raw_output - last_output) / BASE_DT, -TURN_MAX_A, TURN_MAX_A); // in/s/s
    double output = last_output + acceleration * BASE_DT;

    // Constrain the maximum velocity
    // Maximum velocity we can have in order to decelerate to min_veloc with max_accel
    double vmax = sqrt(sq(BASE_MIN_V) + 2 * TURN_MAX_A * fabs(error));

    // Constrain this theoretical maximum with the given bounds
    vmax = clamp(vmax, BASE_MIN_V + EPSILON, 100);
    last_output = (output = clamp(output, -vmax, vmax));

    // Constrain the minimum velocity
    output = ithreshold(output, TURN_MIN_V);
    output = clamp(output / MOTOR_PERCENT_TO_IN_PER_SEC, -80, 80);

    // 
    if (fabs(error) < 1) {
      leftDrive.moveVoltage(0);
      rightDrive.moveVoltage(0);
      brake_cycles += 1;
    } else {
        double volt = 12000*(output/100);
        leftDrive.moveVoltage(volt);
        rightDrive.moveVoltage(-volt);
        brake_cycles = 0;
    }
    if (brake_cycles > 10 && fabs(error) < 1) {
      break;
    }
    pros::delay(BASE_DT*1000);
  }
  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
}

void turn_rel_inertial(double target) {
  turn_absolute_inertial(get_rotation() + target);
}

void inertial_drive(double target, double speed) {
  double LStart = LOdom.get();
  double RStart = ROdom.get();
  //Starting pos
  double angle = Inertial.get();

  // Accumulated error
  double integral_c = 0;
  double last_error;
  double derivative;
  double integral;

  while (true) {
    // Calculate the error
    double error_c = angle - Inertial.get();
    double error1 = target - (LOdom.get()-LStart) * 2.8 * M_PI;
    double error2 = target - (ROdom.get()-RStart) * 2.8 * M_PI;
    double error;
    
    error = (error1 + error2) / 2;

    // Get the turn output
    double raw_output_correct = (TURN_KP * error_c + 0.1 * integral_c); // in/s
    integral_c += error_c * BASE_DT;

    if (fabs(error) > integral_threshold) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    double raw_output = kp * error + ki * integral + kd * derivative; // in/s
    last_error = error;

    // This is the extent to which we want to turn; a low value means no turns
    double factor = 0.1 + fabs(error_c) / 45;
    raw_output_correct *= factor; //old error
    raw_output_correct = error_c*kp_c; //new ange error
    //double correct_output = 2 * clamp(raw_output_correct, -speed, speed);

    if(raw_output > speed) raw_output = speed; //Limit speed
     else if(raw_output < -speed) raw_output = -speed; 
    
     //Normal speed
     if(target!=0){ //fwd correct
      double LVolt = 12000*((raw_output + raw_output_correct)/100);
      double RVolt = 12000*((raw_output - raw_output_correct)/100);
      leftDrive.moveVoltage(LVolt);
      rightDrive.moveVoltage(RVolt);
     }

		if(std::abs(error) <= .5){
      leftDrive.moveVoltage(0);
      rightDrive.moveVoltage(0);
      break;
		}
}
}

void autobalance(){
  float error=0;
  float prevError = 0;
  float derivative = 0;
  float intialError;
  intialError = Inertial_bal.get() - 0; 
  leftDrive.setBrakeMode(AbstractMotor::brakeMode::brake);
  rightDrive.setBrakeMode(AbstractMotor::brakeMode::brake);
  while(true){
    int position = Inertial_bal.get();
    //Propotional
    error = position - 0;
    //Derivative
    derivative = error - prevError;

    if( std::abs(error) < .5)  // we will stop within .5 deg from target
    {
       break;
    }

    double motorPower = error * kp_bal  + derivative * kd_bal;

    pros::delay(5);

    leftDrive.moveVoltage(motorPower);
    rightDrive.moveVoltage(motorPower);

    prevError = error;
  }
  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
}

///* Kalman *///
Kalman1D::Kalman1D(
  double covariance,
  double noise_covariance,
  double initial_state,
  double initial_uncertainty
)
  : Q(covariance), R(noise_covariance)
{
  state = initial_state;
  P = initial_uncertainty;
}

void Kalman1D::update(double measurement) {
  // Find the difference between the filter's current state and the incoming state
  double error = measurement - state;

  // Calculate the extent to which we want to incorporate the incoming value
  // into our current state
  double kalman_gain = P * (1.0 / (P + R));

  // Incorporate the incoming value
  state += kalman_gain * error;

  // Update our uncertainty about our current state
  P = (1.0 - kalman_gain) * P + Q;
}

pros::Mutex heading_mtx;

// Filter to track our rotation
Kalman1D heading_filter(1.0, 0.50, 0.0, 0.0);

void kalmanTask(void* param){
  // Set our initial rotation to 0 regardless of the position of the robot
  Inertial.reset();
  
  while(true) {
    // Obtain our current rotation according to the sensor
    double measured_rotation = Inertial.get();

    // Update the filter
    heading_mtx.take(15);
    heading_filter.update(measured_rotation);
    heading_mtx.give();

    // Wait 5ms before the next update
    pros::delay(5);
  }
}

void initialize_kalman() {
// Start a daemon to update the filter in the background
  pros::task_t my_task = pros::c::task_create(kalmanTask, NULL, TASK_PRIORITY_DEFAULT,
                                TASK_STACK_DEPTH_DEFAULT, "kalman");
}

// Get the current rotation of the robot by querying the state of the filter
double get_rotation() {
  heading_mtx.take(15);
  double rotation = heading_filter.state;
  heading_mtx.give();
  return rotation;
}

///* Util *///
double clamp(double x, double mn, double mx) {
  if (x > mx) {
    return mx;
  } else if (x < mn) {
    return mn;
  }

  return x;
}

double iclamp(double x, double lim) {
  if (x > -lim && x < 0) {
    return -lim;
  } else if (x < lim && x > 0) {
    return lim;
  }
  return x;
}

double ithreshold(double x, double threshold) {
  if (x < 0 && x > -threshold) {
    return -threshold;
  } else if (x > 0 && x < threshold) {
    return threshold;
  } else {
    return x;
  }
}

bool ae(double a, double b) {
  return fabs(a - b) < EPSILON;
}

double sq(double x) {
  return x * x;
}