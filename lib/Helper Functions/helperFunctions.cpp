#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "SPIFFS.h"
#include "helperFunctions.h"

// formatDate takes in ints for month, day and year and returns a formatted string
String formatDate(uint8_t month, uint8_t day, uint8_t yr) {
    String date = "";
    date += String(month);
    date += "/";
    date += String(day);
    date += "/";
    date += String(yr);
    return date;
}

void loadPoints(File file, QueueHandle_t waypointQueue) {
  String line;
  waypoint w;
  Serial.println("File Content:");
  int x = 0;
  while(file.available()){

    line = file.readString();
    for (int i = 0; i < line.length(); i++) {
      if (line[i] == ',') {
        w.wayLat = line.substring(x,i).toDouble();
        w.wayLng = line.substring(i+1).toDouble();

        // create a queue full of waypoints
        if (xQueueSend(waypointQueue, (void *)&w, 10) != pdTRUE) {
            Serial.println("ERROR: Could not put item in waypoint queue.");
          }
        Serial.print("Lat: ");
        Serial.print(w.wayLat, 6);
        Serial.print(" Lng: ");
        Serial.println(w.wayLng, 6);

      } else if (line[i] == '\n') {
        x = i + 1;
      }
    }
  }
}


// formatTime takes in ints for hour, min and second and returns a formatted string
String formatTime(uint8_t hr, uint8_t min, uint8_t sec) {
    String time = "";
    
    if (hr < 5) {
        time += String(hr + 24 - 5);
    } else if (hr < 15){
        time += 0 + String(hr - 5);
    } else {
        time += String(hr - 5);
    }
      
    time += ":";
    
    if (min < 10){
        time += 0 + String(min);
    } else {
        time += String(min);
    }
    
    time += ":";

    if (sec < 10) {
        time += 0 + String(sec);
    } else {
        time += String(sec);
    }

    return time;
}