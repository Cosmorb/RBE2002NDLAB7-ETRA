#include "robot.h"
#include <IRdecoder.h>
#include <HC-SR04.h>
#include <Arduino.h>
#include <Wire.h>
#include "robot.h"
//Creating Ultrasonic object
HC_SR04 hc_sr04(11, 4);

void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();
    hc_sr04.init(ISR_HC_SR04);
    Serial1.begin(115200);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (IR presses, distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);

        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
            distance = 0; // these two lines go in Robot::RobotLoop()
            if(hc_sr04.getDistance(distance)) HandleDistanceReading(distance);
        }
        // AprilTagDatum tag;
        // if(!CheckApproachComplete(hTolerance, dTolerance)) HandleAprilTag(tag);
        if(camera.checkUART(tag)) HandleAprilTag(tag);
    }
}

void ProccesorCOde(const RObotStrcut& nd, Robot& robot) {
    Serial.print("in ProccesorCOde: ");
    Serial.println(nd.caseIDN);

    switch (nd.caseIDN) {
        case 1:
         Serial.println("LED 13 ON");
            digitalWrite(13, HIGH);
            break;

        case 2:
         Serial.println("LED 13 OFF");
            digitalWrite(13, LOW);
            break;

        case 3:
        Serial.println("STOPPING ROBOT");
            break;

        case 4:
        digitalWrite(13,LOW);
            Serial.print("x"); Serial.println(nd.x);
            Serial.print("y"); Serial.println(nd.y);
            Serial.print("theta"); Serial.println(nd.theta);

            robot.FixPoseC(nd);
            break;

        default:
         Serial.print("What did u Do?");
            Serial.println(nd.caseIDN);
            break;
    }
}


void Robot::FixPoseC(const RObotStrcut& nd) {
    SetDestination({nd.x, nd.y, nd.theta});
    robotState = ROBOT_DRIVE_TO_POINT;
}
