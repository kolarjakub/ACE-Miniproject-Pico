#include "fsm.h"

fsm::fsm() : currentState(State::IDLE), previousState(State::IDLE){}

void fsm::setState(State newState){
    if(currentState != newState){
        previousState = currentState;
        currentState = newState;
    }
}

State fsm::getState() const {
    return currentState;
}

State fsm::getPreviousState() const {
    return previousState;
}

bool fsm::isState(State state) const {
    return currentState == state;
}

void fsm::toCalibrationIMU(MPU6500 mpu, imu_t &imu){
    setState(State::CALIBRATION_IMU);
}

void fsm::toLineFollow(){
    setState(State::LINE_FOLLOW);
}

void fsm::toRotate(){
    setState(State::ROTATE);
}

void fsm::toIdle(){
    setState(State::IDLE);
}
