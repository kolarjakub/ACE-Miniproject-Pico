#include "fsm.h"

void fsm::setState(State anewState){
  if (currentState != anewState) {  // if the state chnaged tis is reset
    currentState = anewState;
    tes = millis();
    tis = 0;
  }
}

void fsm::updateTisTes(){
  uint32_t cur_time = millis();   // Just one call to millis()
  tis = cur_time - tes;
}


State fsm::getState() const {
    return currentState;
}