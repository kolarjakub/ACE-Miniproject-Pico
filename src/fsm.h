#ifndef FSM_H
#define FSM_H

#include "robot.h"
#include "MPU6500_Raw.h"

enum class State
{
    IDLE,
    CALIBRATION_IMU,
    LINE_FOLLOW,
    PRE_TURN_FORWARD,
    ROTATE
};

class fsm
{
private:


public:

    State currentState;
    State newState;
    unsigned long tes, tis;

    fsm() : currentState(State::IDLE), newState(State::IDLE) {}

    void setState(State anewState);
    void updateTisTes();
    State getState() const;
    const char* getStateName() const;
};

#endif // FSM_H