// this file will contain various methods to hopefully navigate
// the boat well

#include "navFunctions.h"

double deltaMin = 0, deltaMax = 360, rudMin = 20, rudMax = 160, delta;


// diffControl calculates the difference between the previous 
// heading and current, and adjusts the servo accordingly via a map function
double diffControl(double bearing, double prevBearing) {
    delta = bearing - prevBearing + 180;
     
    // need delta to be between 0 and 360
    if (delta > 360) {
        delta -= 360;
    } else if ( delta < 0) {
        delta += 360;
    }

    // if less than 180, the rudder moves CW, else CCW
    return (delta - deltaMin) * (rudMax - rudMin) / (deltaMax - deltaMin) + rudMin;
}