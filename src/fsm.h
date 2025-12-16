#ifndef FSM_H
#define FSM_H

#include "robot.h"
#include "MPU6500_Raw.h"

enum class State
{
    IDLE,
    CALIBRATION_IMU,
    LINE_FOLLOW,
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

};

#endif // FSM_H