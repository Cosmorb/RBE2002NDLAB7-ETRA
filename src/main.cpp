#include <Arduino.h>
#include "robot.h"
#include <Wire.h>

void ProccesorCOde(const RObotStrcut& nd, Robot& robot);

Robot robot;

void receivedata(int howMany) {
  if (howMany == sizeof(RObotStrcut)) {
    RObotStrcut packet;
      Wire.readBytes((char *)&packet, sizeof(RObotStrcut));
      ProccesorCOde(packet, robot); ;
  } else {
      while (Wire.available()) Wire.read();
  }
}



void setup() 
{
  Serial.begin(115200);

/**
 * If you define __SETUP_DEBUG__ in your .ini file, this line will make the program wait
 * until the Serial is set up so you can debug. WARNING: If you do that, you _must_ open 
 * the Serial Monitor for anything to happen!
 */
#ifdef __SETUP_DEBUG__
  while(!Serial){delay(5);}
#endif

  Wire.begin(0x08);  
  Wire.onReceive(receivedata);
  robot.InitializeRobot();
  
}

void loop() 
{
  robot.RobotLoop();
}
