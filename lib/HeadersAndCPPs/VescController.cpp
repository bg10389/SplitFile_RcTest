#include "TheH-File.h"

// VESC objects on Serial1 and Serial5.
VescUart vesc1;
VescUart vesc2;

void setupVesc() {
    // Initialize VESC1 on Serial1.
    Serial1.begin(115200); 
    vesc1.setSerialPort(&Serial1);
    
    // Initialize VESC2 on Serial5.
    Serial5.begin(115200);
    vesc2.setSerialPort(&Serial5);
}

/////////////////////////////////////////////////////////////////////////////////////
// VESC Current Control (using SBUS channel 1) WITH REVERSE
// SBUS range: ~350..1700, neutral ~990, deadband ±10.
// If above (neutral+deadband), forward current up to safeMaxCurrent.
// If below (neutral-deadband), reverse current down to -safeMaxCurrent.
// Otherwise, coast at 0 A.
/////////////////////////////////////////////////////////////////////////////////////
void updateVescControl() {
    // VESC current control uses SBUS channel 1.
    int ch_vesc = channels[1];
    const int neutral = 990;  // Throttle value at zero position.
    const int deadband = 10;  // Use ±10 counts deadband.
    
    // Use a local safe current limit for VESC control.
    const float safeMaxCurrent = 10.0f;
    
    float currentCommand = 0.0f;
    
    if (ch_vesc > (neutral + deadband)) {
        // Forward throttle: map from (neutral+deadband) up to 1700.
        float forwardRange = (1700.0f - (neutral + deadband));
        currentCommand = (float)(ch_vesc - (neutral + deadband)) / forwardRange;
        currentCommand = constrain(currentCommand, 0.0f, 1.0f);
        currentCommand *= safeMaxCurrent;
    } else if (ch_vesc < (neutral - deadband)) {
        // Reverse throttle: map from (neutral-deadband) down to 350.
        float reverseRange = (float)((neutral - deadband) - 350);
        float proportion = (float)((neutral - deadband) - ch_vesc) / reverseRange;
        proportion = constrain(proportion, 0.0f, 1.0f);
        currentCommand = -proportion * safeMaxCurrent;
    } else {
        // Within deadband, coast.
        currentCommand = 0.0f;
    }
    
    // Issue current command to both VESCs.
    vesc1.setCurrent(currentCommand);
    vesc2.setCurrent(currentCommand);
}
