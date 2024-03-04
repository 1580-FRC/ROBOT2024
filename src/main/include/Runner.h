#include <iostream>
#include <Action.h>
#include <frc/smartdashboard/SmartDashboard.h>
#ifndef RUNNERH
#define RUNNERH                            
class ActionRunner {
    public:
    std::chrono::steady_clock::time_point step_start = std::chrono::steady_clock::now();
    std::vector<Action*> actions;
    int current_step = 0;

    ActionRunner(std::vector<Action*> acts) : actions(acts) {}

    void Update(Robot& robot) {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        auto step_ms =  std::chrono::duration_cast<std::chrono::milliseconds>(now - this->step_start).count();
        if (actions[current_step]->Update(robot)) {
            std::cout << "Action " << current_step << " Has finished IN " << step_ms << std::endl;
            current_step += 1;
            frc::SmartDashboard::PutNumber("CURRENT STEP", current_step);

            step_start = std::chrono::steady_clock::now();
        }else if (actions[current_step]->TimeoutMs() > step_ms) {
            std::cout << "Action " << current_step << " Has TIMED OUT IN " << step_ms << std::endl;
            current_step += 1;
            step_start = std::chrono::steady_clock::now();
        }
    }

};
#endif