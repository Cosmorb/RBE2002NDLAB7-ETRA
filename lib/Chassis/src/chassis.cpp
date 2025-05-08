#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor("L");
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor("R");

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

/**
 * For taking snapshots and raising the flag.
 */
void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    leftMotor.speed = leftMotor.CalcEncoderDelta();
    rightMotor.speed = rightMotor.CalcEncoderDelta();
}

/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}

/**
 * Sets up a hardware timer on Timer4 to manage motor control on a precise schedule.
 * 
 * We set the timer to set an interrupt flag on overflow, which is handled
 * by ISR(TIMER4_OVF_vect) below.
 */
void Chassis::InitializeMotorControlTimer(void)
{
    Serial.println("InitTimer");
    // Disable interupts while we mess with the Timer4 registers
    cli(); 
  
    // Set up Timer4
    TCCR4A = 0x00; // Disable output to pins
    TCCR4B = 0x0A; // Sets the prescaler -- see pp. 167-8 in datasheet
    TCCR4C = 0x00; // Disables output to pins (but see below for buzzer)
    TCCR4D = 0x00; // Normal mode: count up and roll-over

    /**
     * Calculate TOP based on prescaler and loop duration. Note that loop is in integer ms --
     * there may be some rounding. Multiples of 4 ms will be exact.
     */
    uint16_t top = ((CONTROL_LOOP_PERIOD_MS * 16000ul) >> 9) - 1; // divides by 512

    /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 _bits_ in TC4H that we can use to add capacity to TOP.
     * 
     * Note that the maximum period is limited by TOP = 0x3FF. If you want
     * a longer period, you'll need to adjust the pre-scaler.
     * 
     * There is no minumum period, but precision breaks down with low values, 
     * unless you adjust the pre-scaler, but the encoder resolution is limited,
     * so you only want to go so fast.
    */
    uint8_t highbits = top / 256;
    uint8_t lowbits = top - highbits;
    TC4H = highbits; OCR4C = lowbits;

    // Enable overflow interrupt
    TIMSK4 = 0x04; 

    /**
     * Uncommenting the following lines will pipe the timer signal to pin 6, 
     * which controls the buzzer. The pin will toggle at the loop rate, which
     * allows you to check that the timing is correct. It will also make a lot
     * of noise, so do so sparingly.
     */
    // TCCR4C = 0x04
    // pinMode(6, OUTPUT);

    // Re-enable interrupts
    sei(); 

    Serial.println("/InitTimer");
}

void Chassis::InititalizeChassis(void)
{
    InitializeMotorControlTimer();
    InitializeMotors();
}

/**
 * The main Chassis loop.
 */
bool Chassis::ChassisLoop(Twist& velocity)
{
    bool retVal = false;

    if(loopFlag)
    {
        if(loopFlag > 1) Serial.println("Missed an update in Robot::RobotLoop()!");

#ifdef __LOOP_DEBUG__
        Serial.print(millis());
        Serial.print('\n');
#endif

        // motor updates
        UpdateMotors();

        /* Update the wheel velocity so it gets back to Robot. */
        velocity = CalcOdomFromWheelMotion();

        loopFlag = 0;

        retVal = true;
    }

    return retVal;
}

/**
 * Some motor methods.
 */
void Chassis::InitializeMotors(void)
{
    Romi32U4MotorBase::InitializePWMTimerAndInterrupts();

    leftMotor.InitializeMotor();
    rightMotor.InitializeMotor();
}

void Chassis::SetMotorEfforts(int16_t left, int16_t right) 
{
    leftMotor.SetMotorEffortDirect(left); 
    rightMotor.SetMotorEffortDirect(right);
}

void Chassis::UpdateMotors(void)
{
    leftMotor.ControlMotorSpeed();
    rightMotor.ControlMotorSpeed();
}

/**
 * SetWheelSpeeds converts the linear wheel speeds (axes relative to ground) to motor speeds.
 */
void Chassis::SetWheelSpeeds(float leftSpeedCMperSec, float rightSpeedCMperSec)
{
    /** 
     * TODO: Check the code below. You did this in Lab 1, so not repeated here.
     */
    leftMotor.SetTargetSpeed(leftSpeedCMperSec * LEFT_TICKS_PER_CM * CONTROL_LOOP_PERIOD_MS / 1000.);
    rightMotor.SetTargetSpeed(rightSpeedCMperSec * RIGHT_TICKS_PER_CM * CONTROL_LOOP_PERIOD_MS / 1000.);
}

void Chassis::SetTwist(const Twist& twist)
{
    /**
     * TODO: Complete SetTwist() to call SetWheelSpeeds() from target u and omega
     */
     float leftSpeed = twist.u - (twist.omega * ROBOT_RADIUS );
     float rightSpeed = twist.u + (twist.omega * ROBOT_RADIUS );
     SetWheelSpeeds(leftSpeed, rightSpeed);
}



bool Chassis::CheckIMU(float& currentAngleEst)
{
    bool retVal = false;
    // TODO: light LED in pin 13 (n.b., pin 13 is set to output in Chassis::Initialize
    // digitalWrite(13, HIGH); //Light the LED in pin 13
    bool newData = imu.checkForNewData();
    // TODO: unlight LED in pin 13
    // digitalWrite(13, LOW); //Light the LED in pin 13
    float pitch;

    if(newData)
    {
        // TODO: Implement your complementary filter here
        // estimatedPitchAngle at this point holds estimate at k-1

        float Epsilon = 0.0125 ;
        float Kappa = 0.7;
        float prevCorrection = 0; 
        float S = (imu.mdpsPerLSB/1000);


        float PrevbiasY = imu.gyroBias.y;

        float Prediction = prevCorrection + (imu.g.y - imu.gyroBias.y) * imu.mdpsPerLSB /(imu.gyroODR * 1000);
        float Observation = atan2(-(imu.a.x), (imu.a.z))*(180.0/PI);
        float Correction = Prediction + Kappa * (Observation - Prediction); ;
        prevCorrection = Correction;
        imu.gyroBias.y = PrevbiasY-Epsilon*(1./((imu.mdpsPerLSB/1000)*(20/1000.)))*(Observation-Prediction);

        PrevbiasY = imu.gyroBias.y;

        ANGLE.y = Correction;
        estimatedPitchAngle = Correction;
        pitch = Correction;
        
        if(Observation > 1)
        {
            digitalWrite(13, HIGH); //Set pin 13 to HIGH when c is negative
        }else if(Observation < -1)
        {
            digitalWrite(13, HIGH); //Set pin 13 to HIGH when pitch is positive
        }else{
            digitalWrite(13, LOW); //Set pin 13 to LOW when pitch is about 0 (flat) 
        }
    

        #ifdef __IMU_DEBUG__
            TeleplotPrint("a.x", imu.a.x);
            TeleplotPrint("a.y", imu.a.y);
            TeleplotPrint("a.z", imu.a.z);
            TeleplotPrint("g.x", imu.g.x);
            TeleplotPrint("g.y", imu.g.y);
            TeleplotPrint("g.z", imu.g.z);
            TeleplotPrint("bias", imu.gyroBias.y);
            TeleplotPrint("Correction", estimatedPitchAngle);
            TeleplotPrint("offset", imu.offset_A_X);
        #endif
        // update the return value at the end
        currentAngleEst = estimatedPitchAngle;
        retVal = true;
    }
    return retVal;
}

Twist Chassis::CalcOdomFromWheelMotion(void)
{
    Twist velocity;
    /**
     * TODO: Calculate velocities from wheel motion
     */
    
    float leftSpeed = leftMotor.speed / LEFT_TICKS_PER_CM / (CONTROL_LOOP_PERIOD_MS / 1000.0);
    float rightSpeed = rightMotor.speed / RIGHT_TICKS_PER_CM / (CONTROL_LOOP_PERIOD_MS / 1000.0);

    velocity.omega = (rightSpeed - leftSpeed) / (ROBOT_RADIUS * 2.0);

    velocity.u = (leftSpeed + rightSpeed) / 2.0;

#ifdef __NAV_DEBUG__
    TeleplotPrint("u", velocity.u);
    TeleplotPrint("omega", velocity.omega);
#endif

    return velocity;
}
