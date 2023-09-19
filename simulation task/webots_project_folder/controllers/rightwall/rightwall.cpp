#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 3
#define DESIRED_DISTANCE 10// set the desired distance from the wall here

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  DistanceSensor *rightSensor1 = robot->getDistanceSensor("US_right1");
  DistanceSensor *rightSensor2 = robot->getDistanceSensor("US_right2");
  rightSensor1->enable(TIME_STEP);
  rightSensor2->enable(TIME_STEP);

  Motor *leftMotor = robot->getMotor("leftwheelM");
  Motor *rightMotor = robot->getMotor("rightwheelM");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // set up the PID controller with the desired parameters
  double kp = 0.01; // proportional gain
  double ki = 0.00; // integral gain
  double kd = 0.0; // derivative gain
  double set_point = DESIRED_DISTANCE;
  double error_sum = 0;
  double last_error = 0;
  double dt = TIME_STEP / 1000.0;

  while (robot->step(TIME_STEP) != -1) {
    double rightSensorValue1 = rightSensor1->getValue()/10;
    double rightSensorValue2 = rightSensor2->getValue()/10;
    double actual_distance = (rightSensorValue1 + rightSensorValue2) / 2.0;

    // compute the error and update the error_sum
    double error = set_point - actual_distance;
    error_sum += error * dt;

    // compute the error derivative
    double error_derivative = (error - last_error) / dt;
    last_error = error;

    // compute the PID correction
    double correction = kp * error + ki * error_sum + kd * error_derivative;

    // compute the left and right motor speeds using the correction
    double leftSpeed = MAX_SPEED * 0.5 - correction;
    double rightSpeed = MAX_SPEED * 0.5 + correction;
    std::cout<<leftSpeed<<"   "<<rightSpeed<<"  "<<correction<<std::endl;

    // ensure the speeds are within the allowable range
    if (leftSpeed > MAX_SPEED) {
      leftSpeed = MAX_SPEED;
    }
    if (rightSpeed > MAX_SPEED) {
      rightSpeed = MAX_SPEED;
    }
    if (leftSpeed < 0) {
      leftSpeed = 0;
    }
    if (rightSpeed < 0) {
      rightSpeed = 0;
    }

    // set the motor speeds
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  }

  delete robot;
  return 0;
}
