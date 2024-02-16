#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;

void update_dest_deg(){}

void head_to_target(){}

void bypass_obstacle(bool bypass_from_right_side){}

bool should_bypass_direction_be_right(){
    return true;
}

bool should_bypass_obstacle(){
    return true;
}

void bug_controller::setup() {
    krembot.setup();
    target_pos = targetPosMsg.pos;
}

void bug_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    deg = posMsg.degreeX;

        bool should_bypass_from_right_side;
    if (should_bypass_obstacle()){
        should_bypass_from_right_side = should_bypass_direction_be_right();
        bypass_obstacle(should_bypass_from_right_side);
    }
    else {
        head_to_target();
    }
}

