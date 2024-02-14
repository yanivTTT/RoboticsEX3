#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;


void bug_controller::setup() {
    krembot.setup();
    LOG << "started!!!!" << std::endl;
    target_pos = targetPosMsg.pos;
}

void bug_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    deg = posMsg.degreeX;
    krembot.Base.drive(100,0);
    LOG << (pos - target_pos).SquareLength() << std::endl;
    LOG << deg << std::endl;

}

