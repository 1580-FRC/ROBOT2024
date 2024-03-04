#include <chrono>
#include <vector>
#ifndef ACTIONH
#define ACTIONH
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class Robot;

class Action {
    public:

    virtual int TimeoutMs() = 0;

    // true on finish;
    virtual bool Update(Robot& robot) = 0;
    virtual void End(Robot& robot){};
};


#endif
