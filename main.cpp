











#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <cstddef>
#include <utility>
bool RushState = false;
bool Colourfilterboolean = true;
bool combinedState = false; // Tracks the state of both intake and escalator (on/off)
bool StateReverse = false; // Tracks reverse mode for intake and escalator
bool mogoState = false;     // Tracks the state of the mogo clamp
bool liftState = false;     // Tracks the state of the escalator lift
bool doinkerState = true;
static bool brakeToggle = true; // Tracks brake mode
float Blue = 0;
float Red = 240;
float Colour = Blue;
// check these factors- PID VALUES(ALL INCLUDING SLEW), ALL PORT NUMBERS(OR LETTERS), DETERMINING REVERSAL, ALL DRIVETRAIN CONFIG SETTINGS(TRACK WIDTH ETC), ticks for wall stakes.)
// ---------------------------
// Controller Setup
// ---------------------------
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// ---------------------------
// Motor and Pneumatic Setup
// ---------------------------
pros::MotorGroup intakeMotor({16, -18}, pros::MotorGearset::blue);    // Port 16, 600 RPM, not reversed
pros::Motor escalatorMotor((-18), pros::MotorGearset::blue); // Port 11, 600 RPM, not reversed
pros::ADIDigitalOut mogoClamp('A');    
pros::ADIDigitalOut doinker1('B');                           // ADI port 'A' for pneumatic clamp
pros::ADIDigitalOut doinker2('C');  
pros::ADIButton limitSwitch('E');
pros::Motor escalatorLiftMotor((11), pros::MotorGearset::blue);                   // motor port 30 for escalator lift
// ---------------------------
// Motor Groups
// ---------------------------
pros::MotorGroup leftMotors(
    {-7, -10, -12},               // Ports 6, 21, -5 (reversed)
    pros::MotorGearset::blue);  // Blue gearset (600 RPM)
pros::MotorGroup rightMotors(
    {4, 6, 8},                // Ports -3, -8 (reversed), 14
    pros::MotorGearset::blue);  // Blue gearset (600 RPM)
// ---------------------------
// Sensors
// ---------------------------
pros::Imu imu(9);                       // IMU on port 10
pros::Rotation vertical_encoder(15);  
pros::Rotation horizontal_encoder(19);  // Horizontal tracking wheel encoder on port 25
pros::ADIButton distance_sensor('F');
pros::ADIButton distance_sensor2('G');
// ---------------------------
// Tracking Wheel Configuration
// ---------------------------
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontal_encoder, lemlib::Omniwheel::NEW_2, +0.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&vertical_encoder, lemlib::Omniwheel::NEW_2, +0.75);
// ---------------------------
// PID Configuration
// ---------------------------
// lateral motion controller
lemlib::ControllerSettings linearController(
    14, // proportional gain (kP)
    0, // integral gain (kI)
    45, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)






                                        // 15, // proportional gain (kP)
                                        //     0, // integral gain (kI)
                                        //     15, // derivative gain (kD)
                                        //     0, // anti windup
                                        //     0, // small error range, in inches
                                        //     0, // small error range timeout, in milliseconds
                                        //     0, // large error range, in inches
                                        //     0, // large error range timeout, in milliseconds
                                        //     127 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
5.2, // proportional gain (kP)
0, // integral gain (kI)
40, // derivative gain (kD)
0, // anti windup
0, // small error range, in inches
0, // small error range timeout, in milliseconds
0, // large error range, in inches
0, // large error range timeout, in milliseconds
0 // maximum acceleration (slew)




    // 4, // proportional gain (kP)
    //                                           0, // integral gain (kI)
    //                                           20, // derivative gain (kD)
    //                                           0, // anti windup
    //                                           0, // small error range, in inches
    //                                           0, // small error range timeout, in milliseconds
    //                                           0, // large error range, in inches
    //                                           0, // large error range timeout, in milliseconds
    //                                           0 // maximum acceleration (slew)
);
// ---------------------------
// PID Configuration with CLAMP
// ---------------------------
// lateral motion controller
lemlib::ControllerSettings linearControllerCLAMP(15, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            15, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            127 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularControllerCLAMP(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// ---------------------------
// Odometry Sensors Configuration
// ---------------------------
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// ---------------------------
// Drivetrain + Chassis Configuration
// ---------------------------
lemlib::Drivetrain drivetrain(
    &leftMotors,                      // Left motor group
    &rightMotors,                     // Right motor group
    11,                               // Track width (in inches)
    lemlib::Omniwheel::OLD_325,       // 3.25-inch Omniwheel
    431.25,                              // Drivetrain RPM
    8                                 // Horizontal drift compensation
);
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     4, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  4, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain
);
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
lemlib::Chassis chassisMOGO(drivetrain, linearControllerCLAMP, angularControllerCLAMP, sensors, &throttleCurve, &steerCurve);
// ---------------------------
// Movement Functions - Autonomous
// ---------------------------
void moveForward(int inches, int speed) {
    double targetTicks = inches * (360 / (M_PI * 2.75)); // Convert inches to encoder ticks
    leftMotors.tare_position(); // Reset encoder positions
    rightMotors.tare_position();
































    // Move until the left motor reaches the target ticks
    while (fabs(leftMotors.get_position()) < targetTicks) {
        leftMotors.move(speed);   // Move left motors at the given speed
        rightMotors.move(speed);  // Move right motors at the same speed
    }
































    // Stop the motors after reaching the target
    leftMotors.move(0);
    rightMotors.move(0);
}
void turnAngle(int degrees, int speed) {
    double wheelBase = 10.0; // Distance between wheels (in inches)
    double wheelCircumference = M_PI * 2.75; // Wheel circumference (in inches)
















    // Calculate the target ticks for turning based on the robot's wheelbase
    double turnCircumference = M_PI * wheelBase; // Circumference of the turn circle
    double targetDistance = (fabs(degrees) / 360.0) * turnCircumference; // Distance each wheel travels
    double targetTicks = targetDistance * (360 / wheelCircumference); // Convert to encoder ticks
















    leftMotors.tare_position(); // Reset encoder positions
    rightMotors.tare_position();
















    bool turningRight = degrees > 0; // Determine turning direction
















    while (fabs(leftMotors.get_position()) < targetTicks) {
        if (turningRight) {
            leftMotors.move(speed);  // Turn right: left motors move forward
            rightMotors.move(-speed); // Turn right: right motors move backward
        } else {
            leftMotors.move(-speed); // Turn left: left motors move backward
            rightMotors.move(speed);  // Turn left: right motors move forward
        }
    }
















    // Stop the motors after completing the turn
    leftMotors.move(0);
    rightMotors.move(0);
}
void moveForwardPro(int inches, int speed) {
    double targetTicks = inches * (360 / (M_PI * 2.75)); // Convert inches to encoder ticks
    leftMotors.tare_position(); // Reset encoder positions
    rightMotors.tare_position();
















    imu.reset(); // Reset IMU heading to 0 for straight-line correction
















    while (fabs(leftMotors.get_position()) < targetTicks) {
        double imuHeading = imu.get_heading(); // Get current IMU heading
















        // Adjust motor speeds to correct for deviation
        double correction = imuHeading * 1.0; // Proportional gain for correction (tune this value)
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;
















        leftMotors.move(leftSpeed);   // Adjust left motors based on correction
        rightMotors.move(rightSpeed); // Adjust right motors based on correction
    }
















    // Stop the motors after reaching the target
    leftMotors.move(0);
    rightMotors.move(0);
}
void turnAnglePro(int degrees, int speed) {
    imu.reset(); // Reset IMU heading to 0
    bool turningRight = degrees > 0; // Determine turning direction based on sign of degrees
















    while (fabs(imu.get_heading()) < fabs(degrees)) {
        double currentHeading = imu.get_heading(); // Get current IMU heading
















        // Slow down as we approach the target angle for precision
        double adjustmentSpeed = speed * (1 - (fabs(currentHeading) / fabs(degrees)));
        adjustmentSpeed = fmax(adjustmentSpeed, 20); // Minimum speed to prevent stalling
















        if (turningRight) {
            leftMotors.move(adjustmentSpeed);  // Turn right: left motors move forward
            rightMotors.move(-adjustmentSpeed); // Turn right: right motors move backward
        } else {
            leftMotors.move(-adjustmentSpeed); // Turn left: left motors move backward
            rightMotors.move(adjustmentSpeed);  // Turn left: right motors move forward
        }
    }
















    // Stop the motors after completing the turn
    leftMotors.move(0);
    rightMotors.move(0);
}
void toggleIntakeEscalator(bool state) {
    if (state) {
        intakeMotor.move(127);       // Turn on intake
        escalatorMotor.move(127);   // Turn on escalator
    } else {
        intakeMotor.move(0);        // Turn off intake
        escalatorMotor.move(0);     // Turn off escalator
    }
}
void toggleMogoClamp(bool state) {
    mogoClamp.set_value(state); // Set clamp state (true = activate, false = deactivate)
}
// ---------------------------
// Ladybrown Macros
// ---------------------------
// Define motor group and motors
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
pros::Motor lb(-3);
pros::Rotation rotationSensor(5);
const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0,315,1400};
int currState = 0;
int target = 0;
void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
       
    }
   
    target = states[currState];
}
void liftControl() {


    double kp = 1.14;
    double error = target - lb.get_position();
    double velocity = kp * error;
    lb.move(velocity);
   
   
}
// Function to set lift to any custom position (in centidegrees)
void setLiftTarget(int customTarget) {
    target = customTarget;
}
// ---------------------------
// Rerun algorithm
// ---------------------------
// Data structure to store recorded movement
// Struct for movement state
// ---------------------------
// Rerun algorithm
// ---------------------------
// Data structure to store recorded movement
struct Movement {
    double x;
    double y;
    double heading;
    bool mogoState;
    bool doinkerState;
    bool intakeOn;
    bool intakeReverse;
    int duration; // Duration to hold this state in ms
};
// Global variables
std::vector<Movement> recordedMovements;
bool isRecording = false;
bool isPlayingBack = false;
// Start recording or playback
void toggleRecording() {
    if (!isPlayingBack) {
        isRecording = !isRecording;
        if (isRecording) {
            recordedMovements.clear(); // Start fresh recording
            pros::lcd::print(1, "Recording Started");
            controller.print(2, 0, "Recording Started");
        } else {
            pros::lcd::print(1, "Recording Stopped");
            controller.print(2, 0, "Recording Stopped");
        }
    }
}
// Play back the recorded movements
void playBack() {
    if (!isRecording && !recordedMovements.empty()) {
        isPlayingBack = true;
        pros::lcd::print(1, "Playback Started");
        controller.print(2, 0, "Playback Started");




        for (const auto& move : recordedMovements) {
            // Apply motor movements
            chassis.moveToPose(move.x, move.y, move.heading,10000000);
            pros::delay(move.duration);




            // Apply mogo clamp state
            mogoClamp.set_value(move.mogoState);
            // Apply doinker state
            doinker1.set_value(move.doinkerState);
            // Control intake motors
            if (move.intakeOn) {
                intakeMotor.move(127);
                escalatorMotor.move(127);
            } else if (move.intakeReverse) {
                intakeMotor.move(-127);
                escalatorMotor.move(-127);
            } else {
                intakeMotor.move(0);
                escalatorMotor.move(0);
            }
            // Wait for the duration
            pros::delay(move.duration);
        }












        // Stop all motors after playback
        leftMotors.move(0);
        rightMotors.move(0);
        intakeMotor.move(0);
        escalatorMotor.move(0);
        mogoClamp.set_value(false);
        doinker1.set_value(false);
















        pros::lcd::print(1, "Playback Finished");
        isPlayingBack = false;
    }
}
// Record State using Odometry
void recordCurrentState() {
    if (isRecording) {
        Movement move;
        move.x = chassis.getPose().x;
        move.y = chassis.getPose().y;
        move.heading = chassis.getPose().theta;
        move.mogoState = mogoState;
        move.doinkerState = doinkerState;
        move.intakeOn = combinedState;
        move.intakeReverse = StateReverse;
        move.duration = 20;
        recordedMovements.push_back(move);
    }
}
// ---------------------------
// Potentiometer to Scroll through autonomous
// ---------------------------
// Define potentiometer and autonomous modes
pros::ADIAnalogIn autonomousSelector('H'); // Replace 'H' with the port your potentiometer is connected to
// Divide the potentiometer range into equal segments for modes
int potValue = autonomousSelector.get_value();
int selectedAutonomous = potValue / (4096 / 8); // 8 modes (0-7), adjust divisor if you have more modes
// Define autonomous routines
void BLUE_Four_to_Six_Ring() {
    chassis.setPose(0,0,0);
    // chassis.moveToPose(0, 0, 340, 2000, {.lead = 0.5,}, true);
    chassis.moveToPose(18, -35, 332, 2000, {.forwards = false}, true);
    chassis.waitUntilDone();
    toggleMogoClamp(true);
    pros::delay(250);
    toggleIntakeEscalator(true);
    pros::delay(500);
    chassis.moveToPose(18, -29, 220, 1000, {.lead = 0.3}, true);
    chassis.moveToPose(0.0, -52, 180, 2300, {.lead = 0.3}, true);
    pros::delay(500);
    chassis.moveToPose(16, -28, 220, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 60}, true);
    chassis.moveToPose(-15, -42, 180, 2500, {.lead = 0.3}, true);
    pros::delay(1000);
    chassis.moveToPose(-9.5, -50.5, 180, 2500, {.lead = 0.3}, true);
   
}
void RED_Four_to_Six_Ring() {
     // Autonomous code for Left Red Elims
    chassis.setPose(0,0,0);
    chassis.moveToPose(-17, -29, 30, 2000, {.forwards = false, .lead = 0.5,}, true);
    chassis.waitUntilDone();
    toggleMogoClamp(true);
    pros::delay(500);
    toggleIntakeEscalator(true);
    pros::delay(700);
    chassis.moveToPose(-18, -26, 140, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 40}, true);
    chassis.moveToPose(1.25, -50, 180, 2000, {.lead = 0.3}, true);
    pros::delay(500);
    chassis.moveToPose(-18, -26, 140, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 70}, true);
    chassis.moveToPose(11, -50, 200, 2500, {.lead = 0.3, .minSpeed = 30}, true);
    pros::delay(700);
    chassis.moveToPose(-18, -26, 90, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 70}, true);
    chassis.turnToHeading(80, 1000, {.minSpeed = 70});
    chassis.moveToPose(15, -26, 75, 1500, {.lead = 0.3, .minSpeed = 70}, true);
    pros::delay(2000);
    toggleIntakeEscalator(false);
   
    // // Autonomous code for Left Red Elims
    // chassis.setPose(0,0,0);
    // chassis.moveToPose(-18, -28, 30, 2000, {.forwards = false, .lead = 0.5,}, true);
    //  chassis.waitUntilDone(); toggleMogoClamp(true); pros::delay(500);
    //  toggleIntakeEscalator(true); pros::delay(700);
    //  chassis.moveToPose(-18, -26, 140, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 30}, true);
    //  chassis.moveToPose(6.0, -52, 175, 2000, {.lead = 0.3}, true);
    //  pros::delay(500);
    //  chassis.moveToPose(-18, -26, 140, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 60}, true);
    //  chassis.moveToPose(19.0, -52, 185, 2500, {.lead = 0.3, .minSpeed = 30}, true);
    //  pros::delay(500); chassis.moveToPose(-18, -26, 90, 1000, {.forwards = false, .lead = 0.3, .minSpeed = 60}, true); chassis.turnToHeading(80, 1000, {.minSpeed = 60}); chassis.moveToPose(25, -25, 75, 1500, {.lead = 0.3, .minSpeed = 60}, true); pros::delay(2200); toggleIntakeEscalator(false);
 
   




   
    // toggleMogoClamp(true);
    // chassis.moveToPose(-48, 48, 270, 3000, {.horizontalDrift = 8, .lead = 0.3, .minSpeed = 72}, true);
    // chassis.moveToPose(-24, 48, 270, 3000, {.horizontalDrift = 8, .lead = 0.3 }, true);
    // chassis.moveToPose(-48, 72, 315, 3000, {.horizontalDrift = 8, .lead = 0.3 }, true);
    // chassis.moveToPose(-52, 72, 270, 3000, {. = 8, .lead = 0.3 }, true);
}
void BlueSoloAWP() {
    // Autonomous code for Blue SOLO AWP
    chassis.setPose(0,0,0);
    chassis.moveToPose(10.8, 14, 37, 2000, {.minSpeed = 40}, true);
    chassis.waitUntilDone();
    setLiftTarget(1200);
    pros::delay(700);
    chassis.moveToPose(-17, -40, 30, 2000, {.forwards = false, .minSpeed = 70}, true);
    chassis.waitUntilDone();
    toggleMogoClamp(true);
    chassis.waitUntilDone();
    pros::delay(200);
   
    setLiftTarget(0);
    chassis.moveToPose(-32, -51, 200, 2000, {.lead = 0.6, .minSpeed = 120, }, true);
    toggleIntakeEscalator(true);
    chassis.waitUntilDone();
   
    chassis.moveToPose(-17, -35, 200, 2000, {.forwards = false, .minSpeed = 70}, true);
    chassis.moveToPose(-48, -36, 270, 2000, { .minSpeed = 70}, true);
   
    // chassis.moveToPose(-50, -56.6, 270, 2000, {.minSpeed = 120, }, true);
   


    // // pros::delay(1000);
 
    // // // pros::delay(750);
    // // // // chassis.moveToPose(-45, -55, 270, 2000, {.minSpeed = 60}, true);
    // // // // pros::delay(1000);
    // chassis.swingToHeading(75, lemlib::DriveSide::RIGHT,2000, {.minSpeed = 120}, true);
    // // pros::delay(1000);
   
    chassis.moveToPose(-10, -9.0, 90, 2000,{.minSpeed = 127}, true);
    chassis.waitUntilDone();
    toggleMogoClamp(false);
   
    chassis.moveToPose(35, -9.0, 90, 2000,{.minSpeed = 110}, true);
    chassis.waitUntilDone();
    pros::delay(800);
    toggleIntakeEscalator(false);
    chassis.moveToPose(32, -40, 330, 2000,{.forwards = false, .minSpeed = 80}, true);
    chassis.waitUntilDone();
    toggleMogoClamp(true);
    pros::delay(200);
    chassis.moveToPose(0, -24, 315, 2000, {.minSpeed = 80}, true);
    toggleIntakeEscalator(true);


    // chassis.waitUntil(45);
    // pros::delay(100);
    // toggleMogoClamp(false);
   
    // chassis.waitUntilDone();
    // chassis.moveToPose(22.5,-42 , 330, 2000,{.forwards = false, .minSpeed = 127}, true);
    // chassis.waitUntilDone();
    // toggleMogoClamp(true);
    // pros::delay(500);
    // toggleIntakeEscalator(true);
    // chassis.moveToPose(60, -27, 90, 3000, {.minSpeed = 127}, true);
    // chassis.moveToPose(0, -30, 270, 3000, {.forwards = false, .minSpeed = 120}, true);
   




   
   
   
}
void BlueRingRushSixZero() {
   
    chassis.setPose(13, 0,0);
    toggleMogoClamp(false);
    toggleIntakeEscalator(true);
    //chassis.moveToPose(18, 23.5, 10 ,2000, {.minSpeed = 80, .earlyExitRange = 2});
    //chassis.moveToPose(24, 48, 70, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.moveToPoint(22, 40, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToHeading(38, 2000, {.minSpeed = 60});
    pros::delay(000);
    doinker1.set_value(true);
    pros::delay(700);
    toggleIntakeEscalator(false);
    chassis.moveToPose(-2, 34, 90, 2000,{.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    pros::delay(00);
    doinker1.set_value(false);
    toggleMogoClamp(true);
    toggleIntakeEscalator(true);
    pros::delay(200);
    chassis.moveToPoint(28, 34, 2000, {.maxSpeed = 60, .earlyExitRange = 0.5});
    chassis.moveToPose(65, -12, 135, 2000, {.minSpeed = 80, .earlyExitRange = 0.5});
    chassis.waitUntilDone();
    chassis.setPose(0, 0,0);
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
    chassis.moveToPose(0, 0, 0, 2000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
    chassis.turnToHeading(125, 2000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 100000, 1000000000, {.minSpeed = 127});
}
void RedRingRushSixZero(){

    chassis.setPose(-13, 0,0);
    toggleMogoClamp(false);
    toggleIntakeEscalator(true);
    pros::delay(000);
    doinker2.set_value(true);
    //chassis.moveToPose(-16, 23.5, 350 ,2000, {.minSpeed = 80, .earlyExitRange = 2});
    //chassis.moveToPose(-24.5, 49, 340, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.moveToPoint(-22, 41, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToHeading(319, 2000, {.minSpeed = 60});
    chassis.waitUntilDone();
    toggleIntakeEscalator(false);
    chassis.moveToPose(2, 34, 270, 2000,{.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(00);
    doinker2.set_value(false);
    toggleMogoClamp(true);
    toggleIntakeEscalator(true);
    pros::delay(200);
    chassis.moveToPoint(-29, 34, 2000, {.minSpeed = 40, .earlyExitRange = 0.5});
    chassis.moveToPose(-65, -12, 225, 2000, {.minSpeed = 80, .earlyExitRange = 0.5});
    chassis.turnToHeading(225, 2000, {.minSpeed = 80,});
    chassis.waitUntilDone();
    chassis.setPose(0, 0,0);
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});
    chassis.moveToPose(0, 0, 0, 2000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.moveToPose(0, -10, 0, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});
    chassis.turnToHeading(235, 2000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 100000, 1000000000, {.minSpeed = 127});
}
void skills(){
    //setup
    chassis.setPose(0,-53.5,180, false);
    setLiftTarget(1400);
    pros::delay(1000);
    chassis.moveToPose(0, -48, 180, 2000, {.forwards = false});
    chassis.turnToHeading(90, 2000);
    setLiftTarget(-1400);
    pros::delay(1000);
    chassis.moveToPose(-20, -48, 90, 2000, {.forwards = false});
    chassis.waitUntilDone();
    toggleMogoClamp(true);
    pros::delay(500);
    toggleIntakeEscalator(true);
    chassis.moveToPoint(-12, -48, 2000);
    chassis.turnToHeading(337.5, 2000);
    // chassis.moveToPoint(-24, -24, 2000, {.earlyExitRange = 2});
    // chassis.moveToPoint(-36, 0, 2000, {.earlyExitRange = 2});
    // chassis.moveToPoint(-48, 24, 2000, {.earlyExitRange = 2});
    chassis.moveToPoint(-38, 48, 2000, {.earlyExitRange = 2});
    chassis.moveToPoint(-30, 40, 2000, {.forwards = false});
    chassis.turnToHeading(200,2000, {.earlyExitRange = 2});
    chassis.moveToPoint(-44, 0, 2000,{.earlyExitRange = 2});
    chassis.turnToHeading(90, 2000);
}    
//     //setup
//        chassis.setPose(0,11.5,5,0);
//        //RedRingRushSixZero
//        toggleIntakeEscalator(true);
//        pros::delay(1000);
//        toggleIntakeEscalator(false);
//        //1stmogo
//        chassis.moveToPose(0, 24, 0, 2000, {.minSpeed = 60});
//        chassis.turnToHeading(90, 2000);
//        chassis.moveToPose(-23, 23, 90, 2000, {.forwards = false, .minSpeed = 60});
//        chassis.waitUntilDone();
//        toggleMogoClamp(true);
//        pros::delay (500);
//        //1string
//        chassis.turnToHeading(0, 2000);
//        chassis.moveToPose(-23, 55, 0, 2000, {.minSpeed = 60});
//        toggleIntakeEscalator(true);
//        //2ndring
//        chassis.turnToPoint(-48, 53, 2000);
//        chassis.moveToPose(-45, 50, 270, 2000, {.minSpeed = 60});
//        //3rdring
//        chassis.turnToHeading(180, 2000);
//       chassis.moveToPose(-45, 24, 180, 2000, {.minSpeed = 60});
//       chassis.waitUntilDone();
//       pros::delay(2000);
//       //4thring
//       chassis.moveToPose(-49, 5, 180, 2000, {.minSpeed = 60});
//       chassis.waitUntilDone();
//       pros::delay(2000);
//       //back up
//       chassis.moveToPose(-47.5, 25, 180, 2000, {.forwards = false, .minSpeed = 60});
//       //5thring
//       chassis.turnToHeading(270, 2000);
//       chassis.moveToPose(-61.5, 20, 270, 2000, {.minSpeed = 60});
//       pros::delay(2000);
//       //back up
//       chassis.moveToPose(-48, 24, 270, 2000, {.forwards = false, .minSpeed = 60});
//       //corner
//       chassis.turnToHeading(25, 2000);
//       chassis.waitUntilDone();
//       toggleIntakeEscalator(false);
//       chassis.setPose(0, 0, 0);
//       chassis.moveToPose(0, -22, 0, 2000, {.forwards = false, .minSpeed = 60});
//       chassis.waitUntilDone();
//       toggleIntakeEscalator(false);
//       toggleMogoClamp(false);
//       //get out of corner
//       chassis.moveToPose(0, 0, 0, 2000, {.minSpeed = 60,});
//       chassis.turnToHeading(0, 2000);
//       //2ndmogo
//       chassis.setPose(0, 0, 25);
//       chassis.turnToHeading(270, 2000);
//       chassis.moveToPose(62, -14.5, 270, 2000, {.forwards = false, .minSpeed = 60});
//       chassis.moveToPose(78, -14.5, 270, 2000, {.forwards = false,});
//       chassis.waitUntilDone();
//       toggleMogoClamp(true);
//      //1string
//      chassis.turnToHeading(0, 2000);
//      toggleIntakeEscalator(true);
//      chassis.moveToPose(75, 19, 0, 2000);
//      //2ndring
//      chassis.turnToHeading(90, 2000);
//      chassis.moveToPose(103, 19, 90, 2000);
//      //3rdring
//      chassis.turnToHeading(180, 2000);
//      chassis.moveToPose(101, -7, 180, 2000);
//      //4thring
//      chassis.turnToHeading(180, 2000);
//      chassis.moveToPose(101, -22, 180, 2000);
//      //back up
//      chassis.moveToPose(101, -7, 180, 2000, {.forwards = false});
//      //5thring
//      chassis.turnToHeading(90, 2000);
//      chassis.moveToPose(112, -4, 90, 2000);
//      chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 100});
//      //back up
//      //chassis.moveToPose(101, -7, 90, 2000, {.forwards = false});
//      //corner
     
//      chassis.turnToHeading(0, 2000);
//      chassis.waitUntilDone();
//      chassis.setPose(0,0,0);
//      chassis.moveToPose(0, -22, 0, 2000, {.forwards = false, .minSpeed = 100});
//      chassis.waitUntilDone();
//      toggleMogoClamp(false);
//      toggleIntakeEscalator(false);
//      //get out of corner
//      chassis.moveToPose(0, 0, 0, 2000);
//     //    //pickup 1string near ladder
//     //    chassis.turnToHeading(0, 2000);
//     //    chassis.moveToPose(48, 72, 0, 2000);
//     //    chassis.turnToHeading(315, 2000);
//     //    toggleIntakeEscalator(true);
//     //    chassis.moveToPose(24, 96, 315, 2000);
//     //    pros::delay(400); //change as needed
//     //    toggleIntakeEscalator(false);
//        // //3rdmogo
//        // chassis.turnToHeading(135, 2000);
//        // chassis.moveToPose(0, 120, 135, 2000, {.forwards = false,});
//        // toggleMogoClamp(false);
//        // toggleIntakeEscalator(true);
//        // //4thmogo corner
//        // chassis.turnToPoint(144, 144, 2000);
//        // chassis.moveToPose(132, 132, 90, 2000);
//        // //get out of corner
//        // chassis.moveToPose(120, 120, 90, 2000, {.forwards = false,});
//        // //5thmogo corner
//        // chassis.turnToPoint(-144, 144, 2000);
//        // chassis.moveToPose(-144, 144, 90, 2000);

void BlueRingRushFiveOne() {
    
    chassis.setPose(13, 0,0);
    toggleMogoClamp(false);
    toggleIntakeEscalator(true);
    //chassis.moveToPose(18, 23.5, 10 ,2000, {.minSpeed = 80, .earlyExitRange = 2});
    //chassis.moveToPose(24, 48, 70, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.moveToPoint(22, 40, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToHeading(38, 2000, {.minSpeed = 60});
    pros::delay(000);
    doinker1.set_value(true);
    pros::delay(700);
    toggleIntakeEscalator(false);
    chassis.moveToPose(-2, 34, 90, 2000,{.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    pros::delay(00);
    doinker1.set_value(false);
    toggleMogoClamp(true);
    toggleIntakeEscalator(true);
    pros::delay(200);
    chassis.moveToPoint(28, 34, 2000, {.maxSpeed = 60, .earlyExitRange = 0.5});
    chassis.moveToPose(65, -12, 135, 2000, {.minSpeed = 80, .earlyExitRange = 0.5});
    chassis.waitUntilDone();
    chassis.setPose(0, 0,0);
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
    chassis.moveToPose(0, 0, 0, 2000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
    chassis.turnToHeading(125, 2000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    setLiftTarget(315);
    chassis.moveToPoint(0, 43.5, 10000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.turnToHeading(270, 2000);
    chassis.waitUntilDone();
    toggleIntakeEscalator(false);
    escalatorLiftMotor.move_velocity(-20);
    setLiftTarget(1400);

}
void RedRingRushFiveOne() {

    chassis.setPose(-13, 0,0);
    toggleMogoClamp(false);
    toggleIntakeEscalator(true);
    pros::delay(000);
    doinker2.set_value(true);
    //chassis.moveToPose(-16, 23.5, 350 ,2000, {.minSpeed = 80, .earlyExitRange = 2});
    //chassis.moveToPose(-24.5, 49, 340, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.moveToPoint(-22, 41, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToHeading(319, 2000, {.minSpeed = 60});
    chassis.waitUntilDone();
    toggleIntakeEscalator(false);
    chassis.moveToPose(2, 34, 270, 2000,{.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(00);
    doinker2.set_value(false);
    toggleMogoClamp(true);
    toggleIntakeEscalator(true);
    pros::delay(200);
    chassis.moveToPoint(-29, 34, 2000, {.minSpeed = 40, .earlyExitRange = 0.5});
    chassis.moveToPose(-65, -12, 225, 2000, {.minSpeed = 80, .earlyExitRange = 0.5});
    chassis.turnToHeading(225, 2000, {.minSpeed = 80,});
    chassis.waitUntilDone();
    chassis.setPose(0, 0,0);
    chassis.moveToPose(0, -15, 0, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});
    chassis.moveToPose(0, 0, 0, 2000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.moveToPose(0, -10, 0, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});
    chassis.turnToHeading(235, 2000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    setLiftTarget(315);
    chassis.moveToPoint(0, 43.5, 10000, {.minSpeed = 127, .earlyExitRange = 3});
    chassis.turnToHeading(90, 2000);
    chassis.waitUntilDone();
    toggleIntakeEscalator(false);
    escalatorLiftMotor.move_velocity(-20);
    setLiftTarget(1400);
}   



pros::Optical colorSortOptical(3);
void colour_sorting_task() {
    colorSortOptical.set_led_pwm(100); // Turn on LED for better detection


    while (true) {
        int colourSortWaitTime = 100; // Delay in milliseconds
        double hue = colorSortOptical.get_hue(); // Get hue value


        std::string detectedColor;


        // Determine color based on hue ranges
        if (hue >= 0 && hue < 30 || hue >= 330 && hue <= 360) {
            detectedColor = "Red";
        } else if (hue >= 90 && hue <= 150) {
            detectedColor = "Green";
        } else if (hue >= 180 && hue <= 270) {
            detectedColor = "Blue";
        } else {
            detectedColor = "Unknown";
        }


        // Print color to the controller screen
        // controller.print(0, 0, "Colour: %s", detectedColor.c_str());


        pros::delay(colourSortWaitTime); // Wait before next read
    }
}
// ---------------------------
// Initialization
// ---------------------------


void initialize() {
    rotationSensor.reset_position();
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
   
   
    // pros::Task sortingTask(colour_sorting_task);


    pros::Task liftControlTask([]{
            while (true) {
                liftControl();
                pros::delay(10);
            }
    });


    pros::Task AutoClamp([]{
        while (true) {
            if(limitSwitch.get_new_press() == true) {
                mogoClamp.set_value(true);
                pros::delay(10);
            }      //in mm away from mobile goal clamp edge
       
           
        }
});




   
   
//     pros::Task liftControlTask1([]{
//         while (true) {
//             int ladbybrownToggle = 1;
//             if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ladbybrownToggle == 1 ){
//                 lb.move_absolute(0,127);
//                 ladbybrownToggle++;
//                 pros::delay(100);
//                 }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ladbybrownToggle == 3){
//                 lb.move_absolute(1200,127);
//                 ladbybrownToggle = 1;
//                 pros::delay(100);]
//                 }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ladbybrownToggle == 2){
//                 lb.move_absolute(305,127);
//                 ladbybrownToggle ++;
//                 pros::delay(100);
           
//                 }
//         }
// });




   


     // Start task to handle autonomous selection
    pros::Task autonomousSelectorTask([] {
        while (true) {
           
            // Update potentiometer value
            int potValue = autonomousSelector.get_value();
            selectedAutonomous = potValue / (4096 / 8); // Map potentiometer value to mode (0-7)
















            // Display the selected autonomous mode
            pros::lcd::print(3, "Pot Value: %d", potValue);  // Show raw potentiometer value
            pros::lcd::print(4, "Selected Auto: %d", selectedAutonomous); // Show selected mode
            if (selectedAutonomous == 0){
                pros::lcd::print(5, "BLUE_Four_to_Six_Ring"); // Show selected mode
               
                controller.print(0, 0, "BLUE_Four_to_Six_Ring");
               
            }
            if (selectedAutonomous == 1){
                pros::lcd::print(5, "BlueSoloAWP"); // Show selected mode
                controller.print(0, 0, "BlueSoloAWP");
            }
            if (selectedAutonomous == 2){
                pros::lcd::print(5, "BlueRingRushFiveOne"); // Show selected mode
                controller.print(0, 0, "BlueRingRushFiveOne");
            }
            if (selectedAutonomous == 3){
                pros::lcd::print(5, "RedRingRushFiveOne"); // Show selected mode
                controller.print(0, 0, "RedRingRushFiveOne");
            }
            if (selectedAutonomous == 4){
                pros::lcd::print(5, "Skills"); // Show selected mode
                controller.print(0, 0, "Skills");
            }
            if (selectedAutonomous == 5){
                pros::lcd::print(5, "RedRingRushSixZero"); // Show selected mode
                controller.print(0, 0, "RedRingRushSixZero");
            }
            if (selectedAutonomous == 6){
                pros::lcd::print(5, "BlueRingRushSixZero"); // Show selected mode
                controller.print(0, 0, "BlueRingRushSixZero");
            }
            if (selectedAutonomous == 7){
                pros::lcd::print(5, "RED_Four_to_Six_Ring"); // Show selected mode
                controller.print(0, 0, "RED_Four_to_Six_Ring");
            }
















            pros::delay(100); // Update every 100ms
        }
    });
















    // pros::Task colourFilter([]{
    //     if (selectedAutonomous == 0) {
    //         Colour = Blue;
   
    //      }else if (selectedAutonomous == 1) {
    //         Colour = Blue;
         
    //      }else if (selectedAutonomous == 2) {
    //         Colour = Blue;
         
    //      }else if (selectedAutonomous == 3) {
    //         Colour = Red;
         
    //      }else if (selectedAutonomous == 4) {
    //         Colour = Red;
         
    //      }else if (selectedAutonomous == 6) {
    //         Colour = Red;
         
    //      }else if (selectedAutonomous == 7) {
    //         Colour = Red;
         
    //      }
    //             while (true) {
    //                 colorFilter();
    //             }
    //     });
















































    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
































    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
































    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
// ---------------------------
// Autonomous
// ---------------------------
void autonomous() {
     // Lock the selected autonomous mode
    int lockedAutonomous = selectedAutonomous;
    // Run the selected autonomous routine
    switch (lockedAutonomous) {
        case 0: BLUE_Four_to_Six_Ring(); break;
        case 1: BlueSoloAWP(); break;
        case 2: BlueRingRushFiveOne(); break;
        case 3: RedRingRushFiveOne(); break;
        case 4: skills(); break;// left blue elims
        case 5: RedRingRushSixZero(); break;
        case 6: BlueRingRushSixZero(); break;
        case 7: RED_Four_to_Six_Ring(); break;
        default: pros::lcd::print(1, "Invalid Auto"); break;
    }
    // moveForward(36, -127);
    // toggleMogoClamp(true);
    // turnAngle(135, 127);
    // moveForward(32, 127);
    // turnAngle(45, -127);
    // toggleIntakeEscalator(true);
    // moveForward(12, 127);
    // turnAngle(100, -127);
    // moveForward(25, 127);
    // turnAngle(40, 127);
    // moveForward(55, 127);
    // moveForward(20, -127);
    // turnAngle(180, 127);
    // moveForward(25, -127);
    // turnAngle(30, 127);
    // moveForward(90, 127);
    // // skills auton run- 20 points
    // moveForward(24, 127);
    // turnAngle(120, 127);
    // moveForward(80, 127);
    // moveForward(80, -127);
    // turnAngle(120, 127);
    // moveForward(20, -127);
    // turnAngle(120, 127);
}  
// ---------------------------
// Operator Control
// ---------------------------
void opcontrol() {
   
    leftMotors.set_brake_mode(pros::MotorBrake::hold);
    rightMotors.set_brake_mode(pros::MotorBrake::hold);
    while (true) {
       
   
     
        // Check for recording toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            toggleRecording();
        }




















        // Record state if recording
        recordCurrentState();
















        // Trigger playback
        if (!isRecording && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            playBack();
        }
















       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            nextState();
        }
       pros::delay(20);
   
        // ----- Combined Intake and Escalator Control -----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            combinedState = !combinedState; // Toggle combined state
            StateReverse = false;          // Disable reverse when toggling forward
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            StateReverse = !StateReverse; // Toggle reverse mode
            combinedState = false;          // Disable forward when toggling reverse
        }
















       








































        if (combinedState) {
            intakeMotor.move(127);          // Forward
            escalatorMotor.move(127);       // Forward
        } else if (StateReverse) {
            intakeMotor.move(-127);         // Reverse
            escalatorMotor.move(-127);      // Reverse
        } else {
            intakeMotor.move(0);            // Stop
            escalatorMotor.move(0);         // Stop
        }




        lb.set_brake_mode(pros::MotorBrake::hold);




















        // ----- Mogo Clamp Control -----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            mogoState = !mogoState;         // Toggle mogo clamp state
            mogoClamp.set_value(mogoState); // Apply mogo clamp
        }
        // ----- Doinker 1 Control -----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            doinkerState = !doinkerState;         // Toggle mogo clamp state
            doinker1.set_value(doinkerState); // Apply mogo clamp
        }
        // // ----- Brake Mode Toggle -----
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        //     brakeToggle = !brakeToggle;
        //     auto mode = brakeToggle ? pros::MotorBrake::hold : pros::MotorBrake::coast;
        //     leftMotors.set_brake_mode(mode);
        //     rightMotors.set_brake_mode(mode);
        // }
        //ring rush
        // ----- Doinker 2 Control -----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            RushState = !RushState;         // Toggle mogo clamp state
            doinker2.set_value(RushState); // Apply mogo clamp
        }












    // --- double stick arcade---
    // get left y and right x positions
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);




            // move the robot
            // prioritize steering slightly(over throttle)
            chassis.arcade(leftY, rightX, false, 0.75);












        // // ----- Drivetrain Control -----
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // chassis.tank(leftY, rightY);
































        pros::delay(10); // Prevent CPU overuse
































     
































      }








}
void disabled() {}
void competition_initialize() {}
