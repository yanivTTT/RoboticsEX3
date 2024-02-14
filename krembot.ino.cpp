#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;
float threshold = 0.05;

enum State {
    move,
    turn
};
State state = turn;

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
    case State::move:{
        krembot.Led.write(0, 255, 0);
        if ((pos - target_pos).SquareLength() <threshold) {
            krembot.Base.stop();
        }
        else {
            krembot.Base.drive(100, 0);
            
        }
        break;
    }

    case State::turn: {
            krembot.Led.write(255, 0, 0);
            if (((deg - CDegrees(270)).UnsignedNormalize().GetValue() > 1) &&
             ((deg - CDegrees(270)).UnsignedNormalize().GetValue() < 359)) {
                LOG << "CHECKING FOR ANGLE: "<<(deg - CDegrees(270)).UnsignedNormalize() << std::endl;
                krembot.Base.drive(0, 20);
            }
            else {
                krembot.Base.stop();
                state = State::move;
            }
        }
        break;

    default:
        break;
    }
}

