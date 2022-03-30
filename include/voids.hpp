#pragma once
#include "main.h"

using namespace okapi;

void spinIntake();

void stopIntake();

void openClaw();

void closeClaw();

void grabRear();

void releaseRear();

void startBar(int volt = 12);

void stopBar();

void turn_absolute_inertial(double target, bool heavy=false);

void turn_rel_inertial(double target);

void inertial_drive(double target, double speed);

void autobalance();

void initialize_kalman();

double get_rotation();

double clamp(double x, double mn, double mx);

double iclamp(double x, double lim);

double ithreshold(double x, double threshold);

bool ae(double a, double b);

double sq(double x);

class Kalman1D {
private:
  // Process nosie covariance
  const double Q;
  // Measurement noise covariance
  const double R;
  // State uncertainty
  double P;

public:
  // Our internal state
  double state;

  // Initializer for the class
  Kalman1D(double covariance, double noise_covariance, double initial_state, double initial_uncertainty);

  // Update the internal state of the filter given the new measurement value.
  void update(double measurement);

  // Set the state of the filter with complete certainty.
  void set_state(double value);
};
