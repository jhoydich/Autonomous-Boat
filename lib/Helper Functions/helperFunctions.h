#include <Arduino.h>

// waypoint struct
struct waypoint {
  double wayLat;
  double wayLng;
};

// reading struct
struct reading {
  String readingTime;
  String date;
  double lat;
  double lng;
  float depth;
  double rudderAngle;
  double bearing;
  double distance;
  double hdop;
};

// prototypes

String formatDate(uint8_t, uint8_t, uint8_t);

String formatTime(uint8_t, uint8_t, uint8_t);

void loadPoints(File file, QueueHandle_t waypointQueue);