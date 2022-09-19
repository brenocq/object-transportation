#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#define TIME_STEP 32
#define MAX_SPEED 6.28

Robot* robot;
Camera* camera;
Motor* rightMotor;
Motor* leftMotor;

enum State {
    SEARCH_GOAL = 0,
    MOVE_AROUND,
    PUSH,
    BE_A_GOAL,
};
State state = SEARCH_GOAL;

void init() {
    // Camera init
    camera = robot->getCamera("camera");
    camera->enable(256);

    // Motor init
    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

void deinit() { camera->disable(); }

bool canSeeGoal() {
    // Check if goal is visible
    int w = camera->getWidth();
    int h = camera->getHeight();
    const uint8_t* image = camera->getImage();
    for (int x = 0; x < w; x++)
        for (int y = 0; y < h; y++) {
            int r = camera->imageGetRed(image, w, x, y);
            int g = camera->imageGetGreen(image, w, x, y);
            int b = camera->imageGetBlue(image, w, x, y);
            if (g > 127 && r < 127 && b < 127) {
                return true;
            }
        }
    return false;
}

void searchGoal() {
    if(canSeeGoal())
    {
        state = MOVE_AROUND;
        return;
    }

    // If can't see goal, move randomly
    leftMotor->setVelocity(MAX_SPEED*0.1);
    rightMotor->setVelocity(MAX_SPEED*0.1);
}

void moveAround() {
    if(!canSeeGoal())
    {
        state = PUSH;
        return;
    }

    // If can still see the goal, follow wall behavior
    leftMotor->setVelocity(MAX_SPEED*0.5);
    rightMotor->setVelocity(MAX_SPEED*0.5);
}

void push() {
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);
}

void beAGoal() {
    // TODO Change color to green?
}

int main(int argc, char** argv) {
    robot = new Robot();

    init();
    while (robot->step(TIME_STEP) != -1) {
        switch(state) {
            case SEARCH_GOAL:
                searchGoal();
                break;
            case MOVE_AROUND:
                moveAround();
                break;
            case PUSH:
                push();
                break;
            case BE_A_GOAL:
                beAGoal();
                break;
            
        }
    }
    deinit();

    delete robot;
    return 0;
}
