
#include <plans.h>

std::vector<Action *> GetPlanCenter(Robot &robot){
    return std::vector<Action *> {
               new PickupArm(robot),
               new WarmShooter(robot),
               new Shoot(robot),
               new SetIntakeOn(robot),
               new MoveA(robot, 120),
           };
}

// std::vector<Action *> GetLeftCenter(Robot &robot){
//     return std::vector<Action *> {
//                new PickupArm(robot),
//                new WarmShooter(robot),
//                new Shoot(robot),
//                new MoveA(robot, -20),
//                new Turn(robot),
//                new SetIntakeOn(robot),
//                new MoveA(robot, 120),
//            };
// }