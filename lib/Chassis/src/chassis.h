#pragma once

#include <Arduino.h>

#include "chassis-params.h"
#include "utils.h"
#include "LSM6.h"
class Chassis
{
protected:

    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static uint8_t loopFlag;

      //For the IMU
        float estimatedPitchAngle = 0;

        //For IMU
       LSM6 imu;
       LSM6::vector<float> ANGLE;
   

public:
    Chassis(void) {}
    void InititalizeChassis(void);

    /* Where the bulk of the work for the motors gets done. */
    bool ChassisLoop(Twist&);

    /* Needed for managing motors. */
    static void Timer4OverflowISRHandler(void);

public:
    /**
     * SetTwist eliminates the need for DriveAt and TurnAt
     */
    void SetTwist(const Twist& twist);
    Twist CalcOdomFromWheelMotion(void);


        //For IMU
        bool CheckIMU(float& currentAngleEst);
    /**
     * A utility function for converting robot speed to wheel speed. Left 
     * public so that you can test more easily.
     */
    void SetWheelSpeeds(float, float);
    void Stop(void) { SetWheelSpeeds(0, 0); }

protected:
    /**
     * Initialization and Setup routines
     */
    void InitializeMotorControlTimer(void);
    void InitializeMotors(void);
    
    void UpdateMotors(void);
    void SetMotorEfforts(int16_t, int16_t);
};
