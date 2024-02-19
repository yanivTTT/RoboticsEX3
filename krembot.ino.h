//yaniv lukin 213377013
//Ido Trabelssi 316334994
#include <Krembot/controller/krembot_controller.h>

struct PosMsg{
    CVector2 pos;
    CDegrees degreeX;
};

struct TargetPosMsg{
    CVector2 pos;
};

class bug_controller : public KrembotController {
private:
    bool isFirst = true;
	Real robotSize = 0.20;
public:
    PosMsg posMsg;
    TargetPosMsg targetPosMsg;

    ParticleObserver Particle;
    ~bug_controller() = default;
    void setup();
    void loop();

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if ( ! krembot.isInitialized() ) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }
    void ControlStep() override {
        if(isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }
};


REGISTER_CONTROLLER(bug_controller, "bug_controller")
