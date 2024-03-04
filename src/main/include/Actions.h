#include <Action.h>
#ifndef ACTIONSH
#define ACTIONSH
class Shoot : public Action {
    public:
    Shoot(Robot& robot);
    int TimeoutMs() override;

    void End(Robot& robot) override;
    bool Update(Robot& robot) override {return false;};

};

class WarmShooter : public Action {
    public:
    WarmShooter(Robot& robot) ;
    int TimeoutMs() override ;

    bool Update(Robot& robot) override ;

};

class PickupArm : public Action{
    public:
    PickupArm(Robot& robot) ;
    int TimeoutMs() override ;

    bool Update(Robot& robot) override;

    void End(Robot& robot) override ;

    int startAngle;
    int frames = 0;
};


class SetIntakeOn : public Action{
    public:
    SetIntakeOn(Robot& robot) ;
    int TimeoutMs() override ;

    bool Update(Robot& robot) override;
};

class Turn : public Action{
    public:
    Turn(Robot* robot, double angle) ;
    int TimeoutMs() override ;

    bool Update(Robot& robot) override;
};

class MoveA : public Action{
    public:

    MoveA(Robot& robot, int dCm) ;

    int TimeoutMs() override ;

    bool Update(Robot& robot) override;
    void End(Robot& robot) override;

    private:
    int distanceCm;
    int startYaw;
    int startDist;
};
#endif