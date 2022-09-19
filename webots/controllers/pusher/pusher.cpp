#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// entry point of the controller
int main(int argc, char** argv) {
    // create the Robot instance.
    Robot* robot = new Robot();

    Motor* leftMotor = robot->getMotor("leftMotor");
    Motor* rightMotor = robot->getMotor("rightMotor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);

    while (robot->step(TIME_STEP) != -1);

    delete robot;
    return 0; // EXIT_SUCCESS
}
