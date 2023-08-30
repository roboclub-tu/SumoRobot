#include <Wire.h>
#include <Zumo32U4.h>
#include "MyServo.h" 
#include "TurnSensor.h"
#include "RemoteConstants.h"
#include "RemoteDecoder.h"

const uint16_t messageTimeoutMs = 115;
bool messageActive = false;
uint16_t lastMessageTimeMs = 0;
RemoteDecoder decoder;

Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonB button2;
Zumo32U4ButtonC button3;

Servo servo;

bool debugMode = false;
bool lowSpeed = false;

unsigned int lineSensorValues[3];

// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring. 
 uint16_t lineSensorThreshold = 500;

int proxMinValue = 3;

// The speed that the robot uses when backing up.
uint16_t reverseSpeed = 400;

// The amount of time to spend backing up after detecting a
// border, in milliseconds.
const uint16_t reverseTime = 300;

// The speed that the robot uses when turning.
uint16_t turnSpeed = 400;
uint16_t turnPreciseSpeed = 200;

// The speed that the robot usually uses when moving forward.
uint16_t forwardSpeed = 300;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
uint16_t veerSpeedLow = 100;
uint16_t veerSpeedHigh = 300;
 
// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
uint16_t rammingSpeed = 400;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 0;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 4000;

// This enum lists the top-level states that the robot can be in.
enum State
{
  StatePausing,
  StateWaiting,
  StateScanning,
  StateDriving,
  StateBacking,
};

State state = StatePausing;

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the display was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;

bool precise = false;

void setup()
{
if(lowSpeed){
    reverseSpeed = 200;
    turnSpeed = 200;
    forwardSpeed = 200;
    veerSpeedLow = 100;
    veerSpeedHigh = 300;
    rammingSpeed = 100;
}

  servo.attach(6); 
  servo.write(90);
  
  turnSensorSetup();
  
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  motors.flipRightMotor(true);

  changeState(StatePausing);
  Serial.begin(9600);
}

void loop()
{
  decoder.service();
  bool buttonPress = buttonA.getSingleDebouncedPress();

  if (button3.isPressed())
  {
    display.clear();
    button3.waitForRelease();
    lineSensorThreshold+=100;
  }

  if (button2.isPressed())
  {
    display.clear();
    button2.waitForRelease();
    lineSensorThreshold-=100;
  }

  if (state == StatePausing)
  {
    // In this state, we just wait for the user to press button
    // A, while displaying the battery voltage every 100 ms.
    
    servo.write(90);
    motors.setSpeeds(0, 0);

    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("Press A"));
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      display.gotoXY(0, 1);
      display.print(readBatteryMillivolts());
    }
    processRemoteEvents();
    if (decoder.criticalTime())
    {
      // The user pressed button A, so go to the waiting state.
      
      changeState(StateWaiting);
    }
  }
  else if (buttonPress)
  {
    // The user pressed button A while the robot was running, so pause.
    changeState(StatePausing);
  }
  else if (state == StateWaiting)
  {
    // In this state, we wait for a while and then move on to the
    // scanning state.

    motors.setSpeeds(0, 0);

    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      // Display the remaining time we have to wait.
      uint16_t timeLeft = waitTime - time;
      display.gotoXY(0, 0);
      display.print(timeLeft / 1000 % 10);
      display.print('.');
      display.print(timeLeft / 100 % 10);
    }
    else
    {
      // We have waited long enough.  Start moving.
      servo.write(180);
      delay(100);
      changeState(StateDriving);
    }
  }
  else if (state == StateBacking)
  {
    // In this state, the robot drives in reverse.

    if (justChangedState)
    {
      justChangedState = false;
//      display.print(F("back"));
    }

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);

    // After backing up for a specific amount of time, start
    // scanning.
    if (timeInThisState() >= reverseTime)
    {
      if(debugMode){
         motors.setSpeeds(0,0);
         delay(1000);
       }
      changeState(StateScanning);
    }
  }
  else if (state == StateScanning)
  {
    // In this state the robot rotates in place and tries to find its opponent.

    if (justChangedState)
    {
      justChangedState = false;
//      display.print(F("scan"));
    }
    
    turnSensorReset();

    if (scanDir == DirectionRight)
    {
      if(precise){
        motors.setSpeeds(turnPreciseSpeed, -turnPreciseSpeed);
        precise=false;
      }
      else{
        motors.setSpeeds(turnSpeed, -turnSpeed);
      }

      while((int32_t)turnAngle > -(turnAngle45*3))
      {
       int diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
        proxSensors.read();
        if ((proxSensors.countsFrontWithLeftLeds() >= proxMinValue
          || proxSensors.countsFrontWithRightLeds() >= proxMinValue)
          && diff==0)
          {
              break;
          }
        
        turnSensorUpdate();
      }
    }
    else
    {
      if(precise){
        motors.setSpeeds(-turnPreciseSpeed, turnPreciseSpeed);
        precise=false;
      }
      else{
        motors.setSpeeds(-turnSpeed, turnSpeed);
      }
      
       while((int32_t)turnAngle < (turnAngle45*3))
       {
        int diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
        proxSensors.read();
        if ((proxSensors.countsFrontWithLeftLeds() >= proxMinValue
          || proxSensors.countsFrontWithRightLeds() >= proxMinValue)
          && diff==0)
          {
              break;
          }
        
        turnSensorUpdate();
      }
    }

    motors.setSpeeds(0,0);
    changeState(StateDriving);

    if(debugMode){
      delay(2000);
    }
  }
  
  else if (state == StateDriving)
  {
    // In this state we drive forward while also looking for the
    // opponent using the proximity sensors and checking for the
    // white border.

    if (justChangedState)
    {
      justChangedState = false;
//      display.print(F("drive"));
    }

    // Check for borders.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      scanDir = DirectionRight;
      changeState(StateBacking);
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      scanDir = DirectionLeft;
      changeState(StateBacking);
    }

    // Read the proximity sensors to see if know where the
    // opponent is.
    proxSensors.read();
    uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
     display.clear();

    if (sum >= 10 || timeInThisState() > stalemateTime)
    {
      // The front sensor is getting a strong signal, or we have
      // been driving forward for a while now without seeing the
      // border.  Either way, there is probably a robot in front
      // of us and we should switch to ramming speed to try to
      // push the robot out of the ring.
      motors.setSpeeds(rammingSpeed, rammingSpeed);

      // Turn on the red LED when ramming.
      ledRed(1);
    }
    else if (sum == 0)
    {
      // We don't see anything with the front sensor, so just
      // keep driving forward.  Also monitor the side sensors; if
      // they see an object then we want to go to the scanning
      // state and turn torwards that object.

      motors.setSpeeds(forwardSpeed, forwardSpeed);

      if (proxSensors.countsLeftWithLeftLeds() >= 3)
      {
        // Detected something to the left.
        scanDir = DirectionLeft;
//        changeState(StateScanning);
//        precise=true;   
      }

      if (proxSensors.countsRightWithRightLeds() >= 3)
      {
        // Detected something to the right.
        scanDir = DirectionRight;
//        changeState(StateScanning);
//        precise=true;  
      }

      ledRed(0);
    }
    else
    {
      // We see something with the front sensor but it is not a
      // strong reading.
      
      if (diff >= 1)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
      }
      else if (diff <= -1)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
      }
      else
      {
        // Both readings are equal, so just drive forward.
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      ledRed(0);
    }
  }
}

// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

// Changes to a new state.  It also clears the display and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  display.clear();
  displayCleared = true;
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the display should call this function when it updates the
// display.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}
void stopCurrentCommand()
{
  motors.setSpeeds(0, 0);
}
void processRemoteCommand(uint8_t command)
{
  switch(command)
  {
    case 0x0B:
    display.print(F("up"));
    motors.setSpeeds(0, -400);
    break;
  
  case 0x07:
    display.print(F("up"));
    motors.setSpeeds(400, 0);
    break;
  }
}
void processRemoteMessage(const uint8_t * message)
{
  // Print the raw message on the first line of the display, in hex.
  // The first two bytes are usually an address, and the third
  // byte is usually a command.  The last byte is supposed to be
  // the bitwise inverse of the third byte, and if that is the
  // case, then we don't print it.
  
  char buffer[9];
  if (message[2] + message[3] == 0xFF)
  {
    sprintf(buffer, "%02X%02X %02X ",
      message[0], message[1], message[2]);
  }
  else
  {
    sprintf(buffer, "%02X%02X%02X%02X",
      message[0], message[1], message[2], message[3]);
  }
 Serial.println(buffer);

 

  // Make sure the address matches what we expect.
  if (message[0] != remoteAddressByte0 ||
    message[1] != remoteAddressByte1)
  {
    Serial.println("bad addr");
    return;
  }

  // Make sure that the last byte is the logical inverse of the
  // command byte.
  if (message[2] + message[3] != 0xFF)
  {
    Serial.println("bad cmd");
    return;
  }

  stopCurrentCommand();
  processRemoteCommand(message[2]);
}

void processRemoteEvents()
{
  if (decoder.getAndResetMessageFlag())
  {
    // The remote decoder received a new message, so record what
    // time it was received and process it.
    lastMessageTimeMs = millis();
    messageActive = true;
    processRemoteMessage(decoder.getMessage());
  }

  if (decoder.getAndResetRepeatFlag())
  {
    // The remote decoder receiver a "repeat" command, which is
    // sent about every 109 ms while the button is being held
    // down.  It contains no data.  We record what time the
    // repeat command was received so we can know that the
    // current message is still active.
    lastMessageTimeMs = millis();
  }
}
