#include "krembot.ino.h"
#include <cmath>

CVector2 pos;
CVector2 target_pos;
CDegrees deg;
CDegrees desired_angle;
float threshold = 0.05;
int counter = 0;
const int min_rotating_speed = 5;
SandTimer timer;

BumpersRes bumper_pressed;

enum State {
    moveToTarge,
    turnToTarget,
    selectBypassDir,
    calcBypassDeg,
    turnToObstscle,
    avoidObstacle,
    pass_obstacle,
    checkObstaclePassed
};
State state = turnToTarget;

double calculateAngle(CVector2  p1, CVector2 p2){
    double dX = p2.GetX() - p1.GetX();
    double dY = p2.GetY() - p1.GetY();
    return (atan2(dY, dX) * 180 / M_PI) + 180;
}

bool should_bypass_obstacle(Krembot& krembot){
    // if one of the bumpers is pressed go to the turn_to_obstacle state
    auto bumpers_res = krembot.Bumpers.read();
    if (bumpers_res.isAnyPressed()){
        if (bumpers_res.front == BumperState::PRESSED){
            LOGERR << "obstacle on front bumper" << std::endl;
        }
        if (bumpers_res.front_left == BumperState::PRESSED){
            LOGERR << "obstacle on front_left bumper" << std::endl;
        }
        if (bumpers_res.front_right == BumperState::PRESSED){
            LOGERR << "obstacle on front_right bumper" << std::endl;
        }
        if (bumpers_res.left == BumperState::PRESSED){
            LOGERR << "obstacle on left bumper" << std::endl;
        }
        if (bumpers_res.right == BumperState::PRESSED){
            LOGERR << "obstacle on right bumper" << std::endl;
        }
        bumper_pressed = bumpers_res;
        return true;
    }
    return false;
}

bool head_straight(Krembot& krembot, int timer){
    if (counter >= timer) //might be better to use sandtime instad or something like that
    {
        counter = 0;
        krembot.Base.stop();
        LOG << "false" << std::endl;
        return false;
    }
    else {
    krembot.Base.drive(100, 0);
    if (counter > timer)
        counter = 0;
    counter ++;  
    LOG << "counter is: " << counter << std::endl;
    LOG << "true " << timer << std::endl;
    return true;         
    }
}

void head_to_target(Krembot& krembot){
    krembot.Led.write(0, 255, 0);
    int timer = 10;
    if ((pos - target_pos).SquareLength() < threshold) {
        krembot.Base.stop();
    }
    else {
        if (should_bypass_obstacle(krembot))
            state = selectBypassDir;
        
        if (!head_straight(krembot, timer)){
            LOG << "TURNNNNIINGNNGNGG"<< std::endl;
            state = State::turnToTarget;
        }
    }
}

void rotate_to_angle(Krembot& krembot, CDegrees desired_deg){
    int diff = std::min(CDegrees(deg - desired_deg).UnsignedNormalize().GetValue(),
                        CDegrees(desired_deg - deg).UnsignedNormalize().GetValue());
    int rotate_speed = std::min(std::max(min_rotating_speed, diff - 5), 100);
    if ((desired_deg - deg).UnsignedNormalize().GetValue() > 180)
        rotate_speed = rotate_speed * (-1);
    krembot.Base.drive(0, rotate_speed);
    LOG << "diff " << diff << std::endl;
    LOG << "desired_deg - deg " << (desired_deg - deg).UnsignedNormalize().GetValue() << std::endl;
    LOG << "rotating at angular speed of " << rotate_speed << std::endl;
}

CDegrees get_angle_to_target(){
    return CDegrees(calculateAngle(target_pos, pos));
}

void turn_toward_target(Krembot& krembot){
    krembot.Led.write(255, 0, 0);
    auto angle_to_target(get_angle_to_target());
    auto delta_angles = (deg - angle_to_target).UnsignedNormalize();
    if ((delta_angles.GetValue() > 1) && (delta_angles.GetValue() < 359)) {
            LOG << "CURRENT ANGLE: " << deg << std::endl;
            LOG << "ANGLE TO TARGET: " << angle_to_target.GetValue() << std::endl;
            LOG << "DESIRED ANGLE: " << delta_angles << std::endl;
            rotate_to_angle(krembot, angle_to_target);
            return;
    }
    else {
        state = State::moveToTarge;
        krembot.Base.stop();
    }
}
/*
CDegrees calc_desired_bypass_deg(Krembot& krembot, bool should_bypass_direction_be_right){
    CDegrees turn_deg{0.0};
    double dir_effect = should_bypass_direction_be_right ? (-1) : 1;
    CDegrees keep_bumper_pressed_tax(10.0);
    LOGERR << bumper_pressed.front_left << std::endl;
    if (BumperState::PRESSED == bumper_pressed.right){
        turn_deg.SetValue(180.0);
    }
    if (BumperState::PRESSED == bumper_pressed.right && BumperState::PRESSED == bumper_pressed.front_right){
        turn_deg.SetValue(157.0);
    }
    if (BumperState::PRESSED == bumper_pressed.left){
        turn_deg = keep_bumper_pressed_tax;
    }
    if (BumperState::PRESSED == bumper_pressed.left && BumperState::PRESSED == bumper_pressed.front_left){
        turn_deg.SetValue(22.0);
    }
    if (BumperState::PRESSED == bumper_pressed.front_right){
        turn_deg.SetValue(135.0);
    }
    if (BumperState::PRESSED == bumper_pressed.front && BumperState::PRESSED == bumper_pressed.front_right){
        turn_deg.SetValue(112.0);
    }
    if (BumperState::PRESSED == bumper_pressed.front_left){
        turn_deg = CDegrees(45.0);
    }
    if (BumperState::PRESSED == bumper_pressed.front && BumperState::PRESSED == bumper_pressed.front_left){
        turn_deg.SetValue(67.0);
    }
    if (BumperState::PRESSED == bumper_pressed.front){
        turn_deg.SetValue(90.0);
    }
    //turn_deg = turn_deg - keep_bumper_pressed_tax;
    turn_deg = turn_deg * dir_effect;
    auto res_dir = CDegrees(turn_deg + deg).UnsignedNormalize();
    LOGERR << "turn_deg: " << turn_deg << ", res_dir: " << res_dir << std::endl;
    return res_dir;
}*/

void rotate_robot_to_bypass_obstacle(Krembot& Krembot, bool should_bypass_direction_be_right){
    static bool move_rotate_flag = true;
    Krembot.Base.drive(50,0);
    auto bumpers = Krembot.Bumpers.read();
    auto rotate_speed = 20;
    //rotate accordinly
    bool does_front_bumpers_pressed = bumpers.front == BumperState::PRESSED
    || bumpers.front_right == BumperState::PRESSED || bumpers.front_left == BumperState::PRESSED;
    bool does_side_bumper_pressed = should_bypass_direction_be_right ?
        bumpers.left == BumperState::PRESSED : bumpers.right == BumperState::PRESSED;
    if (should_bypass_direction_be_right)
        rotate_speed = rotate_speed * (-1);
    if (does_front_bumpers_pressed){
        Krembot.Base.drive(0,rotate_speed);
        LOG << "front_bumpers_pressed" << std::endl;
    }
    //repeat untill left/right bumper activate while front bumpers not
    if (!(bumpers.front == BumperState::PRESSED) && does_side_bumper_pressed){
        state = State::avoidObstacle;
        LOG << "front bumper not pressed and side bumper pressed" << std::endl;
    }
}

bool should_bypass_direction_be_right(Krembot& krembot){
    return true;
    /*
    auto angle_to_target = CDegrees(calculateAngle(target_pos, pos)).UnsignedNormalize().GetValue();
    if (angle_to_target <= 90 || angle_to_target > 270){
        LOGERR << "bypass obstacle from right" << std::endl;
        return true;
    }
    LOGERR << "bypass obstacle from left" << std::endl;
    return false;
    */
}

// currently doesnt handle corners
void rotate_to_bypass_obstacle(Krembot& krembot, CDegrees bypass_desired_deg){
    krembot.Led.write(255, 0, 0);
    bool rotation_complete = false;
    
    rotation_complete =
    bypass_desired_deg.GetValue() < deg.GetValue() + 1 && bypass_desired_deg.GetValue() > deg.GetValue() - 1;
    if (rotation_complete){
        LOGERR << "turn complete" << std::endl;
        state = State::avoidObstacle;
        krembot.Base.stop();
        return;
    }
    
    rotate_to_angle(krembot, bypass_desired_deg);
}

void bypass_obstacle(Krembot& krembot, bool bypass_from_right_side){
    // in the avoidObsticale keep moving fowerd until the right bunmper is no longer pressed, then resume moveToTarget state
    krembot.Led.write(0, 255, 0);
    auto bumpers_res = krembot.Bumpers.read();
    bool does_front_bumper_pressed = bumpers_res.front == BumperState::PRESSED;
    bool does_side_bumper_pressed = bypass_from_right_side ?
    bumpers_res.left == BumperState::PRESSED || bumpers_res.front_left == BumperState::PRESSED
    : bumpers_res.right == BumperState::PRESSED || bumpers_res.front_right == BumperState::PRESSED;
    if (does_front_bumper_pressed){
        state = State::turnToObstscle;
        bumper_pressed = bumpers_res;
        krembot.Base.stop();
    }
    else {
        LOG << "does_side_bumper_pressed: " << does_side_bumper_pressed << std::endl;
        if (!head_straight(krembot, 5) && !does_side_bumper_pressed){
            state = State::pass_obstacle;
            krembot.Base.stop(); 
        }
    }
}

void pass_the_obstacle(Krembot& Krembot, bool should_bypass_direction_be_right){
    timer.start(5);
    if (!timer.finished()){
        Krembot.Base.drive(100, 0);
    }
    else{
        static bool first_calc_flag = true;
        if (first_calc_flag){
            desired_angle = deg + (should_bypass_direction_be_right ? CDegrees(90) : CDegrees(-30));
            first_calc_flag = false;
        }
        auto delta_deg = CDegrees(desired_angle - deg).SignedNormalize().GetValue();
        if (delta_deg > 1 && delta_deg < 359){
            rotate_to_angle(Krembot, desired_angle);
            return;
        }
        else{
            Krembot.Base.drive(100, 0);
            auto bumpers = Krembot.Bumpers.read();
            bool side_bumpers_pressed = should_bypass_direction_be_right ?
            bumpers.front_left == BumperState::PRESSED || bumpers.left == BumperState::PRESSED
            : bumpers.front_right == BumperState::PRESSED || bumpers.right == BumperState::PRESSED;
            if (bumpers.front == BumperState::PRESSED && side_bumpers_pressed){
                state = State::turnToObstscle;
                return;
            }
            if (!side_bumpers_pressed)
                state = State::turnToTarget;
        }
        
            
    }
    
}

void check_obstacle_passed(Krembot& krembot, bool bypass_from_right_side){
    static bool calced_new_deg = false;
    int move_dir = bypass_from_right_side ? 1 : (-1);
    if (!calced_new_deg){
        desired_angle = desired_angle + CDegrees(move_dir * 90);
        calced_new_deg = true;
        LOGERR << "check whether obstacle passed" << std::endl;
    }
    // rotate to dir - 90
    if (!((desired_angle - deg).UnsignedNormalize().GetValue() < 1 ||
        (desired_angle - deg).UnsignedNormalize().GetValue() > 359))
    {
        rotate_to_angle(krembot, desired_angle);
        return;
    }
    timer.start(100);
    if (timer.finished()){
        auto bumpers = krembot.Bumpers.read();
        if (bumpers.isAnyPressed()){
            LOGERR << "didn't pass obstacle" << std::endl;
            state = State::turnToObstscle;
        } else {
            LOGERR << "passed obstacle" << std::endl;
            state = State::turnToTarget;
        }
        return; 
    } else
        krembot.Base.drive(100, 0);
}

/*
void check_obstacle_passed(Krembot& krembot, bool bypass_from_right_side){
    static bool calced_new_deg = false;
    int move_dir = bypass_from_right_side ? 1 : (-1);
    if (!calced_new_deg){
        desired_angle = desired_angle + CDegrees(move_dir * 90);
        calced_new_deg = true;
        LOGERR << "check whether obstacle passed" << std::endl;
    }
    // rotate to dir - 90
    if (!((desired_angle - deg).UnsignedNormalize().GetValue() < 1 ||
        (desired_angle - deg).UnsignedNormalize().GetValue() > 359))
    {
        rotate_to_angle(krembot, desired_angle);
        return;
    }
    // go straight
    LOG << "heading_straight" << std::endl;

    if (head_straight(krembot, 5)){
        LOGERR << "front: " << (krembot.Bumpers.read().front == BumperState::PRESSED) << std::endl;
        LOGERR << "left: " << (krembot.Bumpers.read().front_left == BumperState::PRESSED) << std::endl;
        LOGERR << "right: " << (krembot.Bumpers.read().front_right == BumperState::PRESSED) << std::endl;
        if (krembot.Bumpers.read().front == BumperState::PRESSED
            || krembot.Bumpers.read().front_left == BumperState::PRESSED
            || krembot.Bumpers.read().front_right == BumperState::PRESSED){
            state = State::turnToObstscle;
            desired_angle = desired_angle - CDegrees(move_dir * 90);
            LOGERR << "obstacle didn't passed" << std::endl;
            calced_new_deg = false;
            counter = 0;
        }
        LOG << "return" << std::endl;
        return;
    }
    // if front bumper a else b
    state = State::moveToTarge;
    LOGERR << "obstacle passed" << std::endl;
    calced_new_deg = false;
}*/

bool is_bumper_pressed(BumpersRes result){
    if (result.isAnyPressed())
    {
        return true;
    }
    return false;
}

void bug0_controller::setup() {
    krembot.setup();
    LOG << "started!!!!" << std::endl;
    target_pos = targetPosMsg.pos;
}

void bug0_controller::loop() {
    krembot.loop();
    
    static bool bypass_from_right;

    pos = posMsg.pos;
    deg = posMsg.degreeX.UnsignedNormalize();
    LOG << "State is: " << state << std::endl;
    LOG << deg << std::endl;
    LOG << "Dis is: "<<(pos-target_pos).SquareLength() << std::endl;
    switch (state)
    {
    // head toward target flow:
    case State::moveToTarge:{
        head_to_target(krembot);
        break;
    }
    case State::turnToTarget: {
        turn_toward_target(krembot);
        break;
    }
    // bypass obstacle flow:
    case State::selectBypassDir: {
        bypass_from_right = should_bypass_direction_be_right(krembot);
        state = State::turnToObstscle;
        break;
    }
    // bypass obstacle flow:
    /*
    case State::calcBypassDeg: {
        desired_angle = calc_desired_bypass_deg(krembot, bypass_from_right);
        state = State::turnToObstscle;
        break;
    }*/
    case State::turnToObstscle: {
        rotate_robot_to_bypass_obstacle(krembot, bypass_from_right);
        break;
    }
    case State::avoidObstacle: {
        bypass_obstacle(krembot, bypass_from_right);
        break;
    }
    case State::pass_obstacle: {
        pass_the_obstacle(krembot, should_bypass_direction_be_right);
        break;
    }
    case State::checkObstaclePassed: {
        check_obstacle_passed(krembot, bypass_from_right);
        break;
    }
    default:
        break;
    }
}

