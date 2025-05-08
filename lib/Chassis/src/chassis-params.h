/**
 * TODO: Adjust these to get good FK updates. They're not horrible, but neither are they good.
 */

const float ROBOT_RADIUS = 14.7/2; //Radius of the robot chassis (from each wheel), in centimeters (Cm)
const float ROBOT_WHEEL_RADIUS = 7.0/2; //Radius of the wheel, in centimeters (Cm)
const float LEFT_TICKS_PER_CM = 1440/(7 * 3.1415926); //Calculated value found in prelab, where 7 Cm is the wheel diameter, and the entire value is in ticks/Cm
const float RIGHT_TICKS_PER_CM = 1440/(7 * 3.1415926); //Calculated value found in prelab, where 7 Cm is the wheel diameter, and the entire value is in ticks/Cm
