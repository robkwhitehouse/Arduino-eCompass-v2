/*
 - Electronic Compass
 - RK Whitehouse May 2022
 
*/

/* Software design
 *  
 *  There are two sets of methods 
 *  
 *  1. Foreground tasks - i.e. user interface tasks
 *  2. Background tasks - run at specific intervals
 *  
 *  The background tasks are run by the "TaskScheduler" library without any
 *  direct user interaction
 *  This is a co-operative (non-preemptive) scheduler so needsto be called often (when not responding to user input)
 *  The scheduler dispatcher is called in the main "loop()" method
 *  
 *  The foreground tasks are implemented as a set of methods that are controlled
 *  via a very simple finite state machine. The states are changed in response 
 *  to user actions. 
 *  
 *  NB All of the above tasks must be non-blocking, otherwise the scheduler 
 *  will never run. If your background tasks are not executing on time it is probably
 *  because some individual task is taking too long.
 *
 * Communication between background and foreground tasks is via a set of global static
 * objects and variables
 * 
 * INterrupts are used by the rotary encoder. Encoder events are pushed to foreground
 * tasks via a FIFO Queue
 * 
 */

/*
 * Hardware design
 * 
 * Runs on an ESP-32 devkit with two devices attached to the standard
 * ESP-32 I2C bus pins
 * 1. A 128 X 64 OLED display
 * 2. An Adafruit BNO-055 sensor module 
 * 
 * There is a rotary encoder with a push button on GPIO pins 4 (button) and 16 & 17 (DAT and CLK)
 */


/* Imported libraries */
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
//#include <cppQueue.h>
#include <TaskScheduler.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <DNSServer.h>
#include "ESP32TimerInterrupt.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "Configuration.h"
#include "Dump.h"
#include "PushButton.hpp"
#include "printstatus.h"


/* Local libs */
#include "StateMachine.hpp"
#include <Encoder.h>
#include "NMEA.hpp"
#include "RotaryMenu.hpp"

#define TELNET_PORT 23
#define MAX_TELNET_CLIENTS 4


#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

#define BNO055_SAMPLERATE_DELAY_MS 100

//GPIO pin definitions
#define ENCODER_DATA 17       //Roraty encoder data input (interrupt)
#define ENCODER_CLK  16       //Rotary encoder clock input
#define ENCODER_BUTTON 4     //Rotary encoder push button (interrupt)
//Encoder scanning intervals
#define TIMER1_INTERVAL_MS        20
#define DEBOUNCING_INTERVAL_MS    20
#define LONG_PRESS_INTERVAL_MS    4000   //Button press of 4 seconds

// Init ESP32 timer 1
ESP32Timer ITimer1(1);

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//Pre-Declare background task methods
void output();
void updateHeading();
void turnOff();
void getCalStatus();
void checkWiFiClients();


/* Declare Global Singleton Objects */

// Background tasks
Task outputTask(1000, TASK_FOREVER, &output);              // do output every 1 seconds
Task updateHeadingTask(500, TASK_FOREVER, &updateHeading); //Read sensor twice per second
Task getCalStatusTask(500, TASK_FOREVER, &getCalStatus);   //Read sensor calibration every 500ms
Task checkWiFiClientsTask(2000, TASK_FOREVER, &checkWiFiClients); //Check WiFi client connections every 2 secs

//Background task scheduler
Scheduler runner;

//Rotary Encoder
Encoder encoder(ENCODER_DATA, ENCODER_CLK);
PushButton pushButton(ENCODER_BUTTON);

//State machine - runs foreground tasks
FSM fsm;   //Finite State Machine

//Foreground states (operating modes)

//Navigation mode (normal operating mode)
void navigationStart();
void navigationUpdate();

class NavigationMode : public FSMstate {
  public:
  NavigationMode() {
    begin = navigationStart;
    update = navigationUpdate;
  }
}navigationMode;

//Settings mode 
void settingsStart();
void settingsUpdate();

class SettingsMode : public FSMstate {
  public:
  SettingsMode() {
    begin = settingsStart;
    update = settingsUpdate;
  }
}settingsMode;

//Enter boat compass offset - the delta between the boat compass and the BNO055 compass
//Allows the sensor to be mounted in any orientation in the boat

short boatCompassOffset = 0; //This value is saved in flash
short boatCompass = 0; //Only valid when in boat compass settings mode - do not use elsewhere
 
void boatCompassStart();
void boatCompassUpdate();

class BoatCompassOffsetMode : public FSMstate {
  public:
  BoatCompassOffsetMode() {
    begin = boatCompassStart;
    update = boatCompassUpdate;
  }
}boatCompassOffsetMode;

//BNO055 sensor module
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;

enum displayMode {splashScreen,runModeDisplay,calibrationMenuScreen,offsetEntryDisplay} currentDisplay;

short heading, targetHeading = 0;
bool targetHeadingMode = false;
//Set up some storage for the NMEA output messages
HSCmessage hsc;
HDMmessage hdm;

const char *ssid = "NavSource";
WiFiServer *telnetServer = NULL;
WiFiClient **telnetClients = {NULL};



/* The main loop is implemented as a simple state machine with the following
 *  states and transitions.
 *  State 0 - Intialise - this is called automatically in the Arduino Setup() procedure but
 *  State 1 - Navigate - normal operation
 *  State 3 - Calibration menu- Display menu and get user selection
 *  State 3.1 - Input boat compass offset
 *  State 3.2 - Restore calibration offsets to sensor from flash
 *  State 3.3 - Peform manual calibration and save offsets to flash
 *  
 *  The following events are recognised by the state machine;
 *  
 *  1. Long button press (2 seconds) transition to Menu mode (from anywhere)
 *  2. Short button press - accept current user input and transition to appropriate state
 *  3. Rotary encoder motion - does not cause any state transition - usage varies between states
 * 
 */


void displayCalStatus() {
  char textBuff[20];
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.setTextColor(SH110X_WHITE);
  display.println("Calibration Status");
  display.setCursor(25,0);
  sprintf(textBuff,"S: %1d G: %1d A: %1d M: %1d",
      calStatus.system, calStatus.gyro, calStatus.accel, calStatus.mag);
  display.println(textBuff);
  display.drawLine(0, 59, 127, 59, SH110X_WHITE);
  display.drawCircle(63, 59, 4, SH110X_WHITE);
  display.display();
 
}

void errorDisplay(char *errorString) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,10);
  display.setTextColor(SH110X_WHITE);
  display.println("System error:");
  display.setCursor(0,25);
  display.println(errorString);
  display.display();
}

void setup() {
  uint8_t system, gyro, accel, mag;
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  char buff[64];
  
  Wire.begin();
  
  Serial.begin(115200);
  delay(1000);
  
  if (!EEPROM.begin(512))
  {
    Serial.println("EEPROM failed to initialise");
    while (true);
  }
  else
  {
    Serial.println("EEPROM initialised");
  }

  //Init OLED display
  display.begin(i2c_Address, true); // Address 0x3C default

  //Display splash screen on OLED
  displayOLEDSplash();

 
  //Initialization of the BNO055
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    errorDisplay("No BNO-055 detected.");
    while (1);
  }

  //Look to see if we have existing sensor calibration offset data in EEPROM
  EEPROM.get(eeAddress, bnoID);

  /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor); //Read the ID from current sensor

  if (bnoID != sensor.sensor_id) //no match
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
   else
  {
    foundCalib = true;
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(bnoID);
    EEPROM.get(eeAddress, calibrationData);

    printSensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    eeAddress += sizeof(calibrationData);
    EEPROM.get(eeAddress,boatCompassOffset);
    Serial.print("Boat Compass Offset = ");
    sprintf(buff,"%03D",boatCompassOffset);
    Serial.println(buff);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  printSensorDetails();

  /* Optional: Display current status */
  printSensorStatus();

 /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (foundCalib){
//    Serial.println("Move sensor slightly to calibrate magnetometers");
//    while (!bno.isFullyCalibrated())
//    {
//      bno.getEvent(&event);
//      delay(BNO055_SAMPLERATE_DELAY_MS);
//    }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */

    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    while ( system < 2 || mag < 2 || accel < 1 ) {
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        printCalStatus();
        displayCalStatus();

        /* New line for the next sample */
        Serial.println("");
        bno.getCalibration(&system, &gyro, &accel, &mag);

        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("Calibration Offsets: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  printSensorOffsets(newCalib);

  saveOffsetsToFlash();

  pushButton.reset();


  // -- We have a working sensor so set up the task schedule
  runner.init();
  runner.addTask(outputTask);
  runner.addTask(updateHeadingTask);
  runner.addTask(getCalStatusTask);
  runner.addTask(checkWiFiClientsTask);
  

  //Startup the Wifi access point
  WiFi.softAP(ssid);
  telnetClients = new WiFiClient*[4];
  for(int i = 0; i < 4; i++)
  {
    telnetClients[i] = NULL;
  }
  
  telnetServer = new WiFiServer(23);
  telnetServer->begin();
  IPAddress myAddr = WiFi.softAPIP();
  Serial.print("My IP Address = ");
  Serial.println(myAddr);

  //Start the background tasks
  outputTask.enable();
  updateHeadingTask.enable();
  getCalStatusTask.enable();
  checkWiFiClientsTask.enable();

   
  //Set up inital foreground mode
  fsm.currentState = &navigationMode; 
}

unsigned loopCounter;
long loopStart, totalLoopTime=0;

void loop() {
  
  loopStart = micros();

  //Run the background tasks
  runner.execute();
  //Now run the current foreground task
  fsm.runState();
  //kick the deBounce timer
  deBounceTimer.tick();

  totalLoopTime += micros() - loopStart;
  loopCounter++;

}

//Definition of background tasks

//Output the heading as an NMEA message over WiFi
void output() {
  char buff[128];
  
  sprintf(buff, "Current heading: %03d deg.\n", heading);
  Serial.print(buff);  
/*
  if ( loopCounter > 64 ) {
    loopCounter = 0;
    Serial.print("Average loop time: ");
    Serial.println(totalLoopTime >> 6);
    Serial.println("");
    totalLoopTime = 0;
  }
*/
  
  hdm.update(heading);
  if (targetHeadingMode) hsc.update(targetHeading);
  for ( int i=0; i<MAX_TELNET_CLIENTS; i++ ) {
    if ( telnetClients[i] != NULL ) {
      telnetClients[i]->println(hdm.msgString);
      if (targetHeadingMode) {
        telnetClients[i]->println(hsc.msgString);
      }
    }
  }
}

//Update the heading from the BNO055
void updateHeading() {
          /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;
    heading += boatCompassOffset;
    if (heading > 359) heading -= 359;
    else if (heading < 0) heading += 360;
      
}  

//Get calibration status from the BNO055
void getCalStatus() {
  bno.getCalibration(&calStatus.system, &calStatus.gyro, &calStatus.accel, &calStatus.mag);
}

//Look for new WiFi clients
void checkWiFiClients() {
  WiFiClient tempClient = telnetServer->available();   // listen for incoming clients

  if (tempClient) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
     for (int i=0; i<MAX_TELNET_CLIENTS; i++ ) {
      if ( telnetClients[i] == NULL ) {
        WiFiClient* client = new WiFiClient(tempClient);
        telnetClients[i] = client;
        break;
      }
    }

  }
}

//Foreground methods (states)

void displayOLEDSplash()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20,0);
  display.setTextColor(SH110X_WHITE);
  display.println("E.A.S.T.");
  display.setCursor(25,25);
  display.setTextSize(1);
  display.println("Audio Compass");
  display.setCursor(25,40);
  display.println("Prototype 0.A");

  display.drawLine(0, 59, 127, 59, SH110X_WHITE);
  display.drawCircle(63, 59, 4, SH110X_WHITE);
  display.display();

}

// Display the current heading on the OLED display and 
// Get the required course from the user
//

void navigationStart() {
  pushButton.reset();
  encoder.readAndReset();
  Serial.println("Starting Navigation mode");
}

void navigationUpdate()
{
  char buff[64];
  long diff;
  PushButton::ButtonPress buttonPress;
  
  //Set up run mode OLED display  - display fixed items
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("  Current    Target  ");
  display.drawLine(64, 0, 64, 32, SH110X_WHITE);
  display.drawLine(0, 32, 127, 32, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0,37);
  display.println("Calibration Status");

  //Display data fields
  display.setTextSize(2);
  display.setCursor(14,12);
  sprintf(buff,"%03d", heading);
  display.println(buff);

  if ( targetHeadingMode == true ) {
    //Look for changes in the encoder position
    long encoderPosition = encoder.readAndReset();
    if (encoderPosition != 0) {
      targetHeading += encoderPosition/2;
      if (targetHeading > 359) targetHeading -= 359;
      else if (targetHeading < 0) targetHeading += 360;
    }
    sprintf(buff,"%03d", targetHeading);
  } else strcpy(buff,"OFF");
  
  display.setCursor(78,12); //Target heading field
  display.println(buff);
  display.setCursor(5,50); //Sensor calibration status
  display.setTextSize(1);
  sprintf(buff,"S:%1d G:%1d A:%1d M:%1d",
      calStatus.system, calStatus.gyro, calStatus.accel, calStatus.mag);
  display.println(buff);
  
  display.display();

  //check for button press
  buttonPress = pushButton.read();
  pushButton.reset();

  switch (buttonPress) {
    case PushButton::NONE: // Do nothing
      break; 
    case PushButton::SHORT:
      Serial.println("SHORT press");
      targetHeadingMode = !targetHeadingMode; //Toggle Target Heading on/off
      break;
    case PushButton::LONG:  //switch to options menu
      fsm.currentState = &settingsMode;
      break;
  } 
}

//Setup display for settings mode
void settingsStart() {
    pushButton.reset();
    encoder.readAndReset();
    calibrationMenu.show();
    Serial.println("Starting Settings mode");
}


//Handle user input while displaying settings menu
void settingsUpdate()
{
   //Look for changes in the encoder position
  long encoderPosition = encoder.readAndReset();
  if (encoderPosition != 0) {
    Serial.println(encoderPosition);
    calibrationMenu.show(calibrationMenu.currentSelection + encoderPosition/2);
  }

  //Has the knob (button) been pressed?
  if (pushButton.read() != PushButton::NONE) { //get current selection and switch modes accordingly
    pushButton.reset();
    switch ( calibrationMenu.currentSelection ) {
      case 0:
        //Enter boat compass heading
        fsm.currentState->end();
        fsm.currentState = &boatCompassOffsetMode;
        break;
      case 1: 
        //Restore saved offsets
        restoreOffsetsFromFlash();
        displayConfirmationScreen();
        break;
      case 2:
        //Save offsets to flash
        saveOffsetsToFlash();
        displayConfirmationScreen();
        break;
      case 3:
        //Do full manual recalibration
        break;
      case 4:
        //cancel and return to navigation
        fsm.currentState->end();
        fsm.currentState = &navigationMode;
        break;
    }
  }
}

//Update boat compass offset (will be saved to flash)

void boatCompassStart() {
  
  boatCompass = heading - boatCompassOffset;
  if (boatCompass < 0) boatCompass += 360;
  else if (boatCompass > 359) boatCompass -= 359;
  pushButton.reset();
  encoder.readAndReset();
  Serial.println("Starting Boat Compass mode");
}

void boatCompassUpdate () {
  char buff[64];
  PushButton::ButtonPress buttonPress;
  long encoderPosition;

  encoderPosition = encoder.readAndReset();
  if (encoderPosition != 0) {
    boatCompass += encoderPosition/2;
    if (boatCompass > 359) boatCompass -= 359;
    else if (boatCompass < 0) boatCompass += 360;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Boat Compass Offset");
  display.drawLine(0, 12, 127, 12, SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(48,12);
  sprintf(buff,"%03d", boatCompass);
  display.println(buff);
  display.setCursor(78,12);
  sprintf(buff,"%03d", boatCompassOffset);
  display.println(buff);
  display.drawLine(0,40,127,40, SH110X_WHITE);
 
  display.display();

  //check for button press
  buttonPress = pushButton.read();
  pushButton.reset();

  switch (buttonPress) {
    case PushButton::NONE: // Do nothing
      break; 
    case PushButton::SHORT:
    case PushButton::LONG: 
      if (boatCompass > heading) boatCompassOffset =  boatCompass - heading;
      else boatCompassOffset = heading - boatCompass;
      if (boatCompassOffset > 180) {
        (boatCompassOffset -= 180) * -1; 
      }
      //return to calibration menu
      fsm.currentState = &settingsMode;
      break;
  }
}

void saveOffsetsToFlash() {

  
  Serial.println("\n\nStoring calibration data to EEPROM...");

  int eeAddress = 0;
  bno.getSensor(&sensor);
  bno.getSensorOffsets(calibrationData);
  long bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, calibrationData);

  eeAddress += sizeof(adafruit_bno055_offsets_t);
  EEPROM.put(eeAddress, boatCompassOffset);
  
  EEPROM.commit();
  Serial.println("Data stored to Flash.");

  Serial.println("\n--------------------------------\n");
  
}

void displayConfirmationScreen() {
    //Display confirmation
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Operation complete.");
  display.drawLine(0, 50, 127, 50, SH110X_WHITE);
  display.display();
  delay(2000);
}

//Restore offsets previously saved to Flash

void restoreOffsetsFromFlash() {
  long eeAddress, bnoID;
  adafruit_bno055_offsets_t newCalib;

  eeAddress = 0;
    //Look to see if we have existing sensor calibration offset data in EEPROM
  EEPROM.get(eeAddress, bnoID);

  /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor); //Read the ID from current sensor

  if (bnoID != sensor.sensor_id) //no match
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
   else
  {
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(bnoID);
    EEPROM.get(eeAddress, newCalib);

    printSensorOffsets(newCalib);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(newCalib);

    Serial.println("\n\nCalibration data loaded into BNO055");

    eeAddress += sizeof(newCalib);
    EEPROM.get(eeAddress,boatCompassOffset);
  }
}
