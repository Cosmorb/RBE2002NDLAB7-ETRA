/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
const int DESIRED_DIST = 5600;

void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */
    currPose.x = currPose.x + (0.02 * twist.u * cos(currPose.theta));
    currPose.y = currPose.y + (0.02 * twist.u * sin(currPose.theta));
    
    currPose.theta = currPose.theta + (0.02 * twist.omega); //Default time loop was 0.02s

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    bool xGoal = sqrt((destPose.x - currPose.x) * (destPose.x - currPose.x)) <= 1.0;
    bool yGoal = sqrt((destPose.y - currPose.y) * (destPose.y - currPose.y)) <= 1.0;

    if (xGoal && yGoal) {
        retVal = true;
        Serial.print("Reached destination!");
    }
    digitalWrite(13,LOW);
    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
       
        /**
         * TODO: Add your IK algorithm here. 
         */
         float targetDistance = sqrt(pow((destPose.x - currPose.x), 2) + pow((destPose.y - currPose.y), 2)); //Finding distance error
         
         float targetAngle = atan2(destPose.y - currPose.y, destPose.x - currPose.x);
         
         float distanceTheta = targetAngle - currPose.theta; //Finding theta error
         if (distanceTheta > PI) {
            distanceTheta = distanceTheta - 2 * PI;
         }
         if (distanceTheta < -PI) {
            distanceTheta = distanceTheta + 2 * PI;
         }

         float kpDist = 0.25; //0.15 to 0.25 works well
         float kpTheta = 0.85; //0.85 to 0.9 works well

         
        float u = kpDist * targetDistance;
        float omega = kpTheta * distanceTheta ;
        

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
        // Serial.print("Target Angle: ");
        // Serial.print(targetAngle);
        // Serial.print(distanceTheta);
        // Serial.print("targePose");
        // Serial.print(targetDistance);
        // Serial.print("currPose: ");
        // Serial.print(currentDistance);
#endif

        /**
         * TODO: Call chassis.SetTwist() to command the motion, based on your calculations above.
         */
         
        chassis.SetTwist(Twist(u, 0, omega));
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
     chassis.Stop();
     if (avoidObstacle == true) {
        avoidObstacle = false;
        destPose = originalPose;
        SetDestination(destPose);
     } else {
        chassis.Stop();
        robotState = ROBOT_IDLE;
        digitalWrite(13, LOW);
     }
    // AprilTagDatum tag;
    // if (camera.readTag(tag)) {
    //     EnterApproachingState();
    //     digitalWrite(13, LOW);
    // } else {
    //     EnterSearchingState();
    //     digitalWrite(13, LOW);
    // }
}

void Robot::HandleDistanceReading(float distance) 
{
    
    // if (robotState == ROBOT_DRIVE_TO_POINT) 
    // {
    //     // Serial.print(distance);
    //     // 
    //     if (distance < 15 && avoidObstacle == false) {
    //         // Serial.print("Obstacle detected!");
    //         // chassis.Stop();
    //         // robotState = ROBOT_IDLE;
    //         // digitalWrite(13, LOW);

    //         originalPose = destPose;
    //         avoidObstacle = true;

    //         //Depending on the robot's position, it will move either more x or y
    //         if (currPose.x > currPose.y) {
    //             Pose newWayPoint(currPose.x + 5, currPose.y + 15, currPose.theta);
    //             SetDestination(newWayPoint);
    //         } else {
    //             Pose newWayPoint(currPose.x + 15, currPose.y + 5, currPose.theta);
    //             SetDestination(newWayPoint);
    //         }
    //     }
    // }
}


//For the camera
void Robot::HandleAprilTag(const AprilTagDatum& tag)
{
//You may want to comment some of these out when you’re done testing.
// Serial.print("Tag: ");
// Serial.print(tag.id); Serial.print('\t');
// Serial.print(tag.cx); Serial.print('\t');
// Serial.print(tag.cy); Serial.print('\t');
// Serial.print(tag.h); Serial.print('\t');
// Serial.print(tag.w); Serial.print('\t');
// Serial.print(tag.rot); //rotated angle; try turning the tag
// Serial.print('\n');

/** TODO: Add code to handle a tag in APPROACHING and SEARCHING states. */


// Check both states
//In searching
// if we see a tag, switch to approaching state, change LED too here lol

//If appraoching, call checkApproachComplete() to see if we are close enough to the tag

if (robotState == ROBOT_DRIVE_TO_POINT) { //If checkUART returns true and driving to destination, change to approaching state
    EnterApproachingState();
}

else if (robotState == ROBOT_SEARCHING) { //If checkUART returns true and is currenly searching for a tag, change to approaching state
    EnterApproachingState();
}

else if (robotState == ROBOT_APPROACHING) {
    if (CheckApproachComplete(5, 50)) { //If checkApproachComplete returns true, this is called meaning switch to idle state
        EnterIdleState();
        Serial.println("-> IDLE");
        chassis.Stop();
        digitalWrite(13, LOW); //Turn off LED when Romi is close to tag
    } else {
        float fkp = 0.2;
        float xkp = 0.01;
        float u = fkp * sqrt((DESIRED_DIST - (int)tag.h * (int)tag.w));
        // Serial.print((DESIRED_DIST - (int)tag.h * (int)tag.w));
        float omega = xkp * ((int)tag.cx - 160/2); 
        // if (u > 0.2) u = 0.2;
        // if (u < -0.2) u = -0.2;
        chassis.SetTwist(Twist(u, 0, omega)); //Set the Romi to move towards the tag

        //Testing
        int headingError = abs((int)tag.cx - 160/2); //Finding the x error from tag to camera
        int distError = abs(DESIRED_DIST - ((int)tag.w * (int)tag.h)); //Finding the y error from tag to camera
        Serial.print("Heading error:");
        Serial.print(headingError);
        Serial.print('\t');
        Serial.print("Distance error:");
        Serial.print(distError);
        Serial.print('\t');
        }
    }
}

void Robot::EnterSearchingState(void)
{
/** TODO: Set Romi to slowly spin to look for tags. */

Serial.println("-> SEARCHING");
robotState = ROBOT_SEARCHING;
chassis.SetTwist(Twist(0, 0, 0.5)); //Set the Romi to spin slowly to look for tags
// 
}


void Robot::EnterApproachingState(void)
{
/**
* TODO: Turn on the LED when the Romi finds a tag. For extra points,
* blink out a number that corresponds to the tag ID (must be non-blocking!).
* Be sure to add code (elsewhere) to turn the LED off when the Romi is
* done aligning.
*/

Serial.println("-> APPROACHING");
robotState = ROBOT_APPROACHING;
digitalWrite(13, HIGH); //Turn on LED when tag is found


// AprilTagDatum tag;

// // blinks out the tag ID
// if(millis() - lastBlinkTime > blinkInterval) {
//     digitalWrite(13, HIGH);
//     lastBlinkTime = millis();
// } else if (millis() - lastBlinkTime > blinkInterval / 2) {
//     digitalWrite(13, LOW);
//     lastBlinkTime = millis();
// }
}

/** Note that the tolerances are in integers, since the camera works
* in integer pixels. If you have another method for calculations,
* you may need floats.
*/
bool Robot::CheckApproachComplete(int headingTolerance, int distanceTolerance)
{
    // AprilTagDatum tag;
    // const int threshold = 3000;
    // if (tag.h * tag.w > threshold) {
    //     return true;
    //     digitalWrite(13, LOW);
    // }

/** TODO: Add code to determine if the robot is at the correct location. */

int headingError = abs((int)tag.cx - 160/2); //Finding the x error from tag to camera
int distError = abs(DESIRED_DIST - ((int)tag.w * (int)tag.h)); //Finding the distance from tag to camera


if (headingError <= headingTolerance && distError <= distanceTolerance) {
    Serial.println("Heading error:");
    Serial.println(headingError);
    Serial.println("ID:");
    Serial.println(tag.id);
    return true;
}

return false;
}

//Handler for climbing up the slope
