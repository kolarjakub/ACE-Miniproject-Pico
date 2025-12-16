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
    State currentState;
    State previousState;

public:
    fsm() : currentState(State::IDLE), previousState(State::IDLE){}
    
    void setState(State newState);
    State getState() const;
    State getPreviousState() const;

    bool isState(State state) const;

    void toIdle();
    void toCalibrationIMU(MPU6500 mpu, imu_t &imu);
    void toLineFollow();
    void toRotate();
};

#endif // FSM_H