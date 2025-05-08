#pragma once

#include <Arduino.h>
#include "chassis.h"
#include<openmv.h>
#include "IRdecoder.h"
#include "HC-SR04.h"

struct RObotStrcut {
    uint8_t caseIDN;
    float x;
    float y;
    float theta;
};



class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_AUTO;

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;
    

    //For storing original pose and avoiding obstacles
    Pose originalPose;
    bool avoidObstacle = false;
    
    
    //For the ultrasonic and keeping track of distance result
    float distance;
    //For filtering
    #define N_DIST 10
    int counter;
    float dist_readings[N_DIST]; //To store the data readings
    uint8_t dist_index = 0; //To keept track of the data index

    //For the camera
    bool tagDectected = false;
    OpenMV camera;
    AprilTagDatum tag;
    float prevU = 0;

    float lastBlinkTime;
    float blinkInterval;
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);
    void DRAWFORME(void);
    void FixPoseC(const RObotStrcut& nd);

    

protected:

    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
    void HandleDistanceReading(float);

    // /* Camera methods. */
    void HandleAprilTag(const AprilTagDatum& tag);
    void EnterSearchingState(void);
    void EnterApproachingState(void);
    bool CheckApproachComplete(int headingTolerance, int distanceTolerance);
    
};
