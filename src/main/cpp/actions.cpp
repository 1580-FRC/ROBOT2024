#include <Robot.h>
#include <Action.h>

Shoot::Shoot(Robot &robot)
{
    robot.IntakeToShooter();
}
int Shoot::TimeoutMs() 
{
    return 1500;
}

void Shoot::End(Robot &robot) 
{
    robot.IntakeOff();
    robot.loaded = false;
    robot.flexShoot.Set(0);
}

WarmShooter::WarmShooter(Robot &robot)
{
    robot.Shoot();
}
int WarmShooter::TimeoutMs() 
{
    return 1500;
}

bool WarmShooter::Update(Robot &robot) 
{
    return false;

}

PickupArm::PickupArm(Robot &robot) : startAngle(robot.ArmAngle())
{
}
int PickupArm::TimeoutMs() 
{
    return 2000;
}

bool PickupArm::Update(Robot &robot) 
{
    robot.SetArmPow(0.2 + frames * 0.01);
    frames+=1;

    if (robot.ArmAngle() - this->startAngle > 3.0) {
        return true;
    }

    return false;

}

void PickupArm::End(Robot &robot) 
{
    robot.SetArming(RESTING_ARM_ANGLE);
}


SetIntakeOn::SetIntakeOn(Robot &robot)
{
    robot.IntakeOn(true);
}
int SetIntakeOn::TimeoutMs() 
{
    return 20;
}

bool SetIntakeOn::Update(Robot &robot) 
{
    return false;

}

Turn::Turn(Robot *robot, double angle)
{
    robot->SetRotating(angle);
}
int Turn::TimeoutMs() 
{
    return 1500;
}

bool Turn::Update(Robot &robot) 
{
    return false;
}

MoveA::MoveA(Robot &robot, int dCm) : distanceCm(dCm), startYaw(robot.gyro.GetYaw()), startDist(robot.distCm)
{
}

int MoveA::TimeoutMs() 
{
    return distanceCm / 100 * 3;
}

void MoveA::End(Robot &robot)
{
    robot.Move(0, 0, 0);
}

bool MoveA::Update(Robot &robot) 
{
    // robot.Move(robot)
    double diffYaw = robot.gyro.GetYaw() - startYaw;
    double yawRatio = diffYaw / 90.0;

    double total_power = 1.0;

    double right = total_power * (0.5 - yawRatio / 2.0);
    double left = total_power - right;

    robot.Move(right, left, 0.35 * sgn(distanceCm));

    // if (robot.loaded) {
    //     return true;
    // }

#if DIST_SENSOR
    if (distanceCm > 0)
    {

        if (robot.distCm - this->startDist > distanceCm)
        {
            return true;
        }
    }
    else
    {
        if (robot.distCm - this->startDist < distanceCm)
        {
            return true;
        }
    }

#endif

    return false;
}
