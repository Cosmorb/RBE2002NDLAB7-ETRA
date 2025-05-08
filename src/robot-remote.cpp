/**
 * robot-remote.cpp implements features of class Robot that are related to processing
 * remote control commands. It also manages modes. You might want to trim away some of 
 * the code that isn't needed later in the term.
 */
#include "robot.h"

#include <ir_codes.h>
#include <IRdecoder.h>

/**
 * IRDecoder decoder is declared extern in IRdecoder.h (for ISR purposes), 
 * so we _must_ name it decoder.
 */
#define IR_PIN 17
IRDecoder decoder(IR_PIN);

void Robot::HandleKeyCode(int16_t keyCode)
{ 
    Serial.println(keyCode);

    // Regardless of current state, if ENTER is pressed, go to idle state
    if(keyCode == ENTER_SAVE) { EnterIdleState(); keyString = "";}


    // If STOP/MODE is pressed, it toggles control mode (auto <-> teleop)
    else if(keyCode == STOP_MODE) 
    {
        if(robotCtrlMode == CTRL_AUTO) {EnterTeleopMode(); }
        else if(robotCtrlMode == CTRL_TELEOP) {EnterAutoMode(); }
        EnterIdleState(); // Idle to avoid surprises
        keyString = "";
    }

    if(robotCtrlMode == CTRL_AUTO)
    {
        /**
         * TODO: Add destinations.
         */
        switch(keyCode)
        {
            case NUM_2:
                SetDestination(Pose(60, 0, 0));
                break;
            case NUM_4:
                SetDestination(Pose(30, 30, 0));
                break;
            case NUM_6:
                SetDestination(Pose(30, -30, 0));
                break;
            case NUM_8:
                SetDestination(Pose(0, -0, 0));
                break;
            case NUM_9:
                EnterSearchingState();
                break;
        }
        keyString = "";
    }

    else if(robotCtrlMode == CTRL_TELEOP)
    {
        switch(keyCode)
        {
            case UP_ARROW:
                chassis.SetTwist(Twist(20, 0, 0)); //Default 20
                break;
            case RIGHT_ARROW:
                chassis.SetTwist(Twist(20, 10, -0.39)); //Default 20
                break;
            case DOWN_ARROW:
                break;
            case LEFT_ARROW:
                break;
        }
        keyString = "";
    }    
}

void Robot::EnterTeleopMode(void)
{
    Serial.println("-> TELEOP");
    robotCtrlMode = CTRL_TELEOP;
}

void Robot::EnterAutoMode(void)
{
    Serial.println("-> AUTO");
    robotCtrlMode = CTRL_AUTO;
}
