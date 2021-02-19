// this file will contain various methods to hopefully navigate
// the boat well
#include <Arduino.h>
#include "navFunctions.h"

double deltaMin = -180, deltaMax = 180, rudMin = 20, rudMax = 160, delta, rudAngle;


double mapFn(double output) {
    // if less than 180, the rudder moves CW, else CCW
    return (output - deltaMin) * (rudMax - rudMin) / (deltaMax - deltaMin) + rudMin;
}

// diffControl calculates the difference between the current 
// heading and waypoint, and adjusts the servo accordingly via a map function
double diffControl(double bearing, double prevBearing) {
    delta = bearing - prevBearing;
     
    // need delta to be between -180 and 180 and to return the smallest angle between the two
    if (delta > 180) {
        delta -= 360;
    } else if ( delta < -180) {
        delta += 360;
    }

    return mapFn(delta);
}


double iTerm, prevDelta = 0, dBearing, prevBearing = 0, output;
int sampleTime = 2000;
double kp = 1, ki = .05 * sampleTime, kd = 1000 / sampleTime;

// pidControl takes in the desired and previous bearing and calculates the pid output
// based off of this guys great explanation http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// I would've made one myself but I forgot a lot of my automatic controls class :( sorry Professor Cole
double pidControl(double waypointBearing, double currentBearing) {
    // calculating error
    delta = waypointBearing - currentBearing;

    // need delta to be between -180 and 180 and to return the smallest angle between the two
    if (delta > 180) {
        delta -= 360;
    } else if ( delta < -180) {
        delta += 360;
    }

    iTerm += (ki * delta);
    if(iTerm> rudMax) iTerm= rudMax;
    else if(iTerm< rudMin) iTerm= rudMin;

    dBearing = currentBearing - prevBearing;
    prevBearing = currentBearing;
    // updating value
    prevDelta = delta;

    output = kp * delta + iTerm - kd * dBearing;
    
    if (output > deltaMax) output = deltaMax;
    else if (output < deltaMin) output = deltaMin;
    
    return mapFn(output);
}

