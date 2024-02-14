#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;


void bug_controller::setup() {
    krembot.setup();
    target_pos = targetPosMsg.pos;
}

void bug_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    deg = posMsg.degreeX;
}

