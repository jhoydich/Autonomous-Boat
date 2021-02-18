// internal functions
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SPIFFS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

// external libs
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>

// created libs
#include "navFunctions.h"
#include "helperFunctions.h"



// creating a semaphone
static SemaphoreHandle_t batton;

// queue for readings
static QueueHandle_t readingQueue;
static QueueHandle_t waypointQueue;
static uint8_t readingQueueLen = 15;
static uint8_t waypointQueueLen = 50;

// --- Motor globals ---

// DC Motor
int motor1Pin1 = 12; 
int motor1Pin2 = 13; 
int enable1Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 4;
const int resolution = 8;
int dutyCycle = 250;

// servo object
Servo myservo;

// pwm pin for servo
static const int servoPin = 33;
int pos = 0;

// --- end motor globals ---

// --- gps globals ---

// The TinyGPS++ object
TinyGPSPlus gps;

// gps pins and baud
static const int RXPin = 35, TXPin = 34;
static const uint32_t GPSBaud = 9600;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// global vars for gps that are constantly updated
double lat, lng, hdop;
bool validity = false;
uint8_t day, month;
uint16_t yr;
uint8_t hr, mins, sec;

// --- end gps globals ---



void readGPS(void *parameter) {

  while (1) {

    // read all available gps data
    while (ss.available() > 0) {

      // converting NMEA data
      if (gps.encode(ss.read())) { 

        // getting gps
        if (gps.location.isValid()) { 
          validity = true;
          lat = gps.location.lat();
          lng = gps.location.lng(); 
          hdop = gps.hdop.hdop();
        } else {
          validity = false;
        }

        // getting date
        if (gps.date.isValid()) {
          month = gps.date.month();
          day = gps.date.day();
          yr = gps.date.year();
        }
        
        // getting time (GMT time)
        if (gps.time.isValid()) {
          hr = gps.time.hour();
          mins = gps.time.minute();
          sec = gps.time.second();     
        }   
      }
    }

    // 50 ms delay works the best with the gps
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// handleGPS gets the latest GPS data and then adjusts the navigation
void handleGPS(void *parameter) {
  // about 1 meter
  double uncertainty = .000009;

  // rudder angle vars
  double prevBearing = 1000; // making previous bearing a value it cannot be for initial set up
  bool curValidity;
  uint8_t curDay, curMonth;
  uint16_t curYr;
  uint8_t curHr, curMin, curSec;
  reading r;
  waypoint w;

  // getting first waypoint

  if (xQueueReceive(waypointQueue, (void *)&w, 0) != pdTRUE) {
    // kill motor f'n
  }

  while(1) {
    // Serial.println("In other f'n");
    
    // get latest values
    xSemaphoreTake(batton, portMAX_DELAY);
    curValidity = validity;
    r.lat = lat;
    r.lng = lng;
    r.hdop = hdop;
    curDay = day;
    curMonth = month;
    curYr = yr;
    curHr = hr;
    curMin = mins;
    curSec = sec;
    xSemaphoreGive(batton);

    if (curValidity == true) {
      
      // check if we are at the waypoint, if so, update waypoint
      if (w.wayLat - uncertainty < r.lat && r.lat < w.wayLat + uncertainty && w.wayLng - uncertainty < r.lng && r.lng < w.wayLng + uncertainty ) {
        if (xQueueReceive(waypointQueue, (void *)&w, 0) != pdTRUE) { // kill motors if nothing in queue, we are at the final location
          //TODO: kill motor f'n
        }
      }  

      // updating reading struct
      r.date = formatDate(curMonth, curDay, curYr);   
      r.readingTime = formatTime(curHr, curMin, curSec);        
      r.distance = gps.distanceBetween(r.lat, r.lng, w.wayLat, w.wayLng);
      r.bearing = gps.courseTo(r.lat, r.lng, w.wayLat, w.wayLng);
      

      // if first gps co-ord, update prevBearing
      if (prevBearing == 1000) {
        prevBearing = r.bearing;
      } else {

        // navigation control function
        r.rudderAngle = diffControl(r.bearing, prevBearing);

        prevBearing = r.bearing;

        // write to servo
        myservo.write(r.rudderAngle);

        // sending data into queue
        if (xQueueSend(readingQueue, (void *)&r, 10) != pdTRUE) {
            Serial.println("ERROR: Could not put item in reading queue.");
          }
      }

    } else {
      Serial.println("No readings yet");
    }
  
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

}

void logReadings(void *parameter) {
  reading item;
  
  while (1) {
    if (xQueueReceive(readingQueue, (void *)&item, 0) == pdTRUE) { 
        Serial.println("-----------------");
        Serial.print("Location: ");
        Serial.print(item.lat, 6);
        Serial.print(",");
        Serial.print(item.lng, 6);
        Serial.print(" Date: ");
        Serial.print(item.date);
        Serial.print(" Time: ");
        Serial.println(item.readingTime);
        Serial.print("Bearing: ");
        Serial.print(item.bearing);
        Serial.print(" Distance (m): ");
        Serial.print(item.distance);
        Serial.print(" HDOP: ");
        Serial.println(item.hdop);
        Serial.println("-----------------");
        Serial.println();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ss.begin(9600);

  // creating mutex and queues
  batton = xSemaphoreCreateMutex();
  readingQueue = xQueueCreate(readingQueueLen, sizeof(reading));
  waypointQueue = xQueueCreate(waypointQueueLen, sizeof(waypoint));

  // starting spiffs
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  File file = SPIFFS.open("/gps.csv");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  loadPoints(file, waypointQueue);
  
  file.close();

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  
  // allocating timers (does it need all of these?)
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400);

  // placing rudder in the middle
  myservo.write(90);
  
  // creating tasks for cores to run
  xTaskCreatePinnedToCore(&readGPS, "read GPS", 5000, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(&handleGPS, "handle GPS data", 10000, NULL, 7, NULL, 0);
  xTaskCreatePinnedToCore(&logReadings, "log data", 5000, NULL, 5, NULL, 0);

  //TODO: Start motor function incorporating gps
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, dutyCycle);
  
}

// void loop is technically a task, so long delay gives other f'ns more "time" to run
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}