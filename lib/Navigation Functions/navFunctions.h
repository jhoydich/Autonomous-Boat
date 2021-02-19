
// diffControl calculates the difference between the 
// current bearing and desired and moves the rudder accordingly
double diffControl(double bearing, double prevBearing);

// pidControl used a pid controller to 
double pidControl(double waypointBearing, double currentBearing);