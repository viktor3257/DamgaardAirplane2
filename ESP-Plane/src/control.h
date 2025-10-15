#pragma once

void initControl();
void updateControl();

// Throttle value actually sent to the ESC
extern float g_outputThrottle;
// Desired angles the controller is targeting (degrees)
extern float g_desiredRoll;
extern float g_desiredPitch;
