#include <Arduino.h>

// edge detect (buttons should only register once)
bool rising_edge_detect(bool button_state, bool carry_state){ 

    //if the state switches from low to high, there is a rising edge
    if (carry_state == 0 && button_state == 1){
        return true;
    }
    return false;
}