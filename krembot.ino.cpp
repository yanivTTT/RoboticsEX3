#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;
float threshold = 0.05;
int counter = 0;

enum State {
    moveToTarge,
    turnToTarget,
    turnToObstscle,
    avoidObstacle
};
State state = turnToTarget;

void bypass_obstacle(bool bypass_from_right_side){}

bool should_bypass_direction_be_right(){
    return true;
}

bool should_bypass_obstacle(){
    return true;
}

double calculateAngle(CVector2  p1, CVector2 p2){
    double dX = p2.GetX() - p1.GetX();
    double dY = p2.GetY() - p1.GetY();
    return (atan2(dY, dX) * 180 / M_PI) + 180;


}

void bug0_controller::setup() {
    krembot.setup();
    LOG << "started!!!!1" << std::endl;
    target_pos = targetPosMsg.pos;
}

void bug0_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    deg = posMsg.degreeX;
    LOG << deg << std::endl;
    LOG << "Dis is: "<<(pos-target_pos).SquareLength() << std::endl;
    switch (state)
    {
    case State::moveToTarge:{
        krembot.Led.write(0, 255, 0);
        if ((pos - target_pos).SquareLength() <threshold) {
            krembot.Base.stop();
        }
        else {
            //TODO: if one of the bumpers is pressed go to the turn toobstical state
            if (counter == 10) //might be better to use sandtime instad or something like that
            {
                LOG << "TURNNNNIINGNNGNGG"<< std::endl;
                counter = 0;
                state = State::turnToTarget;
            }
            else {
            krembot.Base.drive(100, 0);
            counter ++;  
            LOG << "counter is: "<< counter << std::endl;              
            }
            
        }
        break;
    }

    case State::turnToTarget: {
            krembot.Led.write(255, 0, 0);
                
            if (((deg - CDegrees(calculateAngle(target_pos, pos))).UnsignedNormalize().GetValue() > 1) &&
             ((deg - CDegrees(calculateAngle(target_pos, pos))).UnsignedNormalize().GetValue() < 359)) {
                LOG << "CHECKING FOR ANGLE: "<<(deg - CDegrees(calculateAngle(target_pos, pos))).UnsignedNormalize() << std::endl;
                LOG << "the angle withtarget is: "<<calculateAngle(target_pos, pos) << std::endl;
                krembot.Base.drive(0, 5);
            }
            else {
                krembot.Base.stop();
                state = State::moveToTarge;
            }
        }
        break;
    //TODO: in the turn to obstacle roatet the robot until the right bumper is pressed
    //TODO: in the avoidObsticale kepp mpving fowerd until the right bunmpers is no longer pressed than resume moveToTarget state
    default:
        break;
    }
}

