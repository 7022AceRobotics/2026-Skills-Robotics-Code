/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Robotics                                                  */
/*    Created:      10/9/2025, 8:14:39 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// Arm Motors
vex::motor MotorArm1 = vex::motor( vex::PORT4, vex::gearSetting::ratio36_1, false );
vex::motor MotorArm2 = vex::motor( vex::PORT3, vex::gearSetting::ratio18_1, true );
vex::servo BaseMotor = vex::servo( Brain.ThreeWirePort.A );

vex::bumper Arm1Bumper = vex::bumper( Brain.ThreeWirePort.H );
vex::bumper Arm2Bumper = vex::bumper( Brain.ThreeWirePort.G );

vex::motor WristMotor1 = vex::motor( vex::PORT11 );
vex::motor WristMotor2 = vex::motor( vex::PORT12 );
vex::motor ClawMotor = vex::motor( vex::PORT1, vex::gearSetting::ratio36_1, false );

vex::motor FLDriveMotor = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor FRDriveMotor = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::motor BLDriveMotor = vex::motor(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor BRDriveMotor = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);

// Fuse Constants
const float fuseGreenMotor1 = -312.4f;
const float fuseGreenMotor2 = -325.6f;
const float fuseGreenWrist1 = -300.0f;
const float fuseGreenWrist2 = 300.0f;

const float fuseYellowMotor1 = -300.2f;
const float fuseYellowMotor2 = -396.0f;
const float fuseYellowWrist1 = -200.0f;
const float fuseYellowWrist2 = 200.0f;

const float fuseBlueMotor1 = -312.4f;
const float fuseBlueMotor2 = -325.6f;
const float fuseBlueWrist1 = -300.0f;
const float fuseBlueWrist2 = 300.0f;

// Transformers Constants
const float transformerBottomMotor1 = -608.8f;
const float transformerBottomMotor2 = -142.4f;
const float transformerBottomWrist1 = -800.0f;
const float transformerBottomWrist2 = 800.0f;

const float transformerTopMotor1 = -467.3f;
const float transformerTopMotor2 = -173.6f;
const float transformerTopWrist1 = -650.0f;
const float transformerTopWrist2 = 650.0f;

const float transformerMotor1 = -547.6f;
const float transformerMotor2 = -172.8f;
const float transformerWrist1 = -740.0f;
const float transformerWrist2 = 740.0f;

// Circuit Boards Constants

// Movement Variables
bool setMovement = false;
bool movingMovement = false;

float MotorArm1Ratio = 60.0f / 12.0f;
float MotorArm2Ratio = 24.0f / 6.0f;

float WristAngle1 = 0.0f;
float WristAngle2 = 0.0f;

float AxisXVelocity = 0.005f; // degrees per second
float AxisYVelocity = 0.005f; // degrees per second

float MovementSpeed = 5.0f; // Speed of movement

bool movementRight = true;

// Arm Variables
float L1 = 10.0f; // Length of Arm 1
float L2 = 10.0f; // Length of Arm 2
float TargetX = 20.0f; // Target X Position
float TargetY = 0.0f; // Target Y Position
float Angle1 = 0.0f; // Calculated Angle for Arm 1
float Angle2 = 0.0f; // Calculated Angle for Arm 2

// Controller
vex::controller Controller = vex::controller();

float clamp ( float value, float min, float max );
void calcAngles (float &angle1, float &angle2);

void calcAngle ()
{
    while (true)
    {
        // Calculate Inverse Kinematics
        float ca1p = Controller.Axis1.position( percentUnits::pct );
        if (movementRight)
            TargetX += ca1p * AxisXVelocity; // Scale movement speed
        else
            TargetX -= ca1p * AxisXVelocity; // Scale movement speed
        float ca2p = Controller.Axis2.position( percentUnits::pct );
        TargetY += ca2p * AxisYVelocity; // Scale movement speed

        TargetX = clamp( TargetX, - (L1 + L2), (L1 + L2) );
        TargetY = clamp( TargetY, 0.0f, (L1 + L2) );

        Brain.Screen.printAt( 10, 80, "Wrist1: %.2f", WristMotor1.position( vex::rotationUnits::deg ) );
        Brain.Screen.printAt( 10, 110, "Wrist2: %.2f", WristMotor2.position( vex::rotationUnits::deg ) );

        calcAngles(Angle1, Angle2);

        // yield to other tasks
        this_thread::sleep_for( 15 );
    }
}

void driveBase ()
{
    
    while (true)
    {
        float ca4 = Controller.Axis1.position( percentUnits::pct );
        float ca2 = Controller.Axis2.position( percentUnits::pct );
        float ca3 = Controller.Axis3.position( percentUnits::pct );
        float ca1 = Controller.Axis4.position( percentUnits::pct );

        FLDriveMotor.spin( vex::directionType::fwd, -ca3 -ca1 -ca4, vex::percentUnits::pct );
        FRDriveMotor.spin( vex::directionType::fwd, -ca3 +ca1 +ca4, vex::percentUnits::pct );
        BLDriveMotor.spin( vex::directionType::fwd, -ca3 +ca1 -ca4, vex::percentUnits::pct );
        BRDriveMotor.spin( vex::directionType::fwd, -ca3 -ca1 +ca4, vex::percentUnits::pct );

    }
}

void moveArm ()
{

    WristMotor1.setVelocity(50, vex::percent);
    WristMotor2.setVelocity(50, vex::percent);

    while (true) 
    {
        // Move Arm Target Position Based on Controller Input
        // MotorArm1.spinToPosition( Angle1 * MotorArm1Ratio, vex::degrees, true );
        // MotorArm2.spinToPosition( Angle2 * MotorArm2Ratio, vex::degrees, true );

        // Arm Control
        if ( Controller.ButtonL1.pressing() )
            BaseMotor.setPosition(-50, vex::degrees );
        else if ( Controller.ButtonL2.pressing() )
            BaseMotor.setPosition( 50, vex::degrees );
        else
            BaseMotor.setPosition( 0, vex::degrees );
        
        if (setMovement || movingMovement) {
            movingMovement = true;

            if ( Controller.ButtonDown.pressing() ) {    //Controller.ButtonUp.pressing()
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(fuseYellowWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseYellowWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseYellowMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(fuseYellowMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(fuseYellowMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseYellowMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(fuseYellowWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseYellowWrist2, vex::degrees, true );
                }
            }
            else if ( Controller.ButtonLeft.pressing() ) {
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(fuseGreenWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseGreenWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseGreenMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(fuseGreenMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(fuseGreenMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseGreenMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(fuseGreenWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseGreenWrist2, vex::degrees, true );
                }
            }
            else if ( Controller.ButtonRight.pressing() ) {
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(fuseBlueWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseBlueWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseBlueMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(fuseBlueMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(fuseBlueMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(fuseBlueMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(fuseBlueWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(fuseBlueWrist2, vex::degrees, true );
                }
            }
            else if ( Controller.ButtonUp.pressing() ) {
                MotorArm1.spinToPosition(-5, vex::degrees, true );
                MotorArm2.spinToPosition(-5, vex::degrees, true );
                WristMotor1.spinToPosition(0, vex::degrees, false );
                WristMotor2.spinToPosition(0, vex::degrees, true );
            }
            else if ( Controller.ButtonX.pressing() ) {
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(transformerTopWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerTopWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerTopMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(transformerTopMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(transformerTopMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerTopMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(transformerTopWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerTopWrist2, vex::degrees, true );
                }
            }
            else if ( Controller.ButtonB.pressing() ) {
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(transformerBottomWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerBottomWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerBottomMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(transformerBottomMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(transformerBottomMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerBottomMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(transformerBottomWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerBottomWrist2, vex::degrees, true );
                }
            }
            else if ( Controller.ButtonY.pressing() ) {
                if ( MotorArm1.position( vex::rotationUnits::deg ) < -300.0f ) {
                    WristMotor1.spinToPosition(transformerWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerWrist2, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerMotor2, vex::degrees, true );
                    MotorArm1.spinToPosition(transformerMotor1, vex::degrees, true );
                }
                else {
                    MotorArm1.spinToPosition(transformerMotor1, vex::degrees, true );
                    MotorArm2.spinToPosition(transformerMotor2, vex::degrees, true );
                    WristMotor1.spinToPosition(transformerWrist1, vex::degrees, false );
                    WristMotor2.spinToPosition(transformerWrist2, vex::degrees, true );
                }
            }

            movingMovement = false;

        }
        else {

            if ( Controller.ButtonDown.pressing() && !Arm1Bumper.pressing() )
                MotorArm1.spin( vex::directionType::fwd, MovementSpeed * MotorArm1Ratio, vex::velocityUnits::rpm );
            else if ( Controller.ButtonUp.pressing() )
                MotorArm1.spin( vex::directionType::rev, MovementSpeed * MotorArm1Ratio, vex::velocityUnits::rpm );
            else
                MotorArm1.stop();

            if ( Controller.ButtonRight.pressing() && !Arm2Bumper.pressing() )
                MotorArm2.spin( vex::directionType::fwd, MovementSpeed * MotorArm2Ratio, vex::velocityUnits::rpm );
            else if ( Controller.ButtonLeft.pressing() )
                MotorArm2.spin( vex::directionType::rev, MovementSpeed * MotorArm2Ratio, vex::velocityUnits::rpm );
            else
                MotorArm2.stop();

            if ( Controller.ButtonX.pressing() ) {
                WristMotor1.spin( vex::directionType::fwd, -50, vex::velocityUnits::rpm );
                WristMotor2.spin( vex::directionType::fwd, 50, vex::velocityUnits::rpm );
            }
            else if ( Controller.ButtonB.pressing() ) {
                WristMotor1.spin( vex::directionType::fwd, 50, vex::velocityUnits::rpm );
                WristMotor2.spin( vex::directionType::fwd, -50, vex::velocityUnits::rpm );
            }
            else if (Controller.ButtonY.pressing()) {
                WristMotor1.spin( vex::directionType::fwd, 50, vex::velocityUnits::rpm );
                WristMotor2.spin( vex::directionType::fwd, 50, vex::velocityUnits::rpm );
            }
            else if (Controller.ButtonA.pressing()) {
                WristMotor1.spin( vex::directionType::fwd, -50, vex::velocityUnits::rpm );
                WristMotor2.spin( vex::directionType::fwd, -50, vex::velocityUnits::rpm );
            }
            else {
                WristMotor1.stop();
                WristMotor2.stop();
            }
        }

        // Allow other tasks to run
        this_thread::sleep_for( 15 );
    }
}

int main() {

    Brain.Screen.printAt( 10, 50, "SKILLS_2026 Test VSCode 3.0" );

    MotorArm1.setBrake( vex::brakeType::hold );
    MotorArm1.setPosition( 0, vex::rotationUnits::deg );

    MotorArm2.setBrake( vex::brakeType::hold );
    MotorArm2.setPosition( 0, vex::rotationUnits::deg );

    WristMotor1.setBrake( vex::brakeType::hold );
    WristMotor1.setPosition( 0, vex::rotationUnits::deg );

    WristMotor2.setBrake( vex::brakeType::hold );
    WristMotor2.setPosition( 0, vex::rotationUnits::deg );

    ClawMotor.spinToPosition( 0, vex::rotationUnits::deg, false );
    ClawMotor.setVelocity( 50, vex::percentUnits::pct );
    ClawMotor.setMaxTorque( 25, vex::percentUnits::pct );
    ClawMotor.setBrake( vex::brakeType::hold );

    Controller.ButtonR1.pressed(
        []()
        {
            ClawMotor.spin( vex::directionType::fwd, 50, vex::percentUnits::pct );
        } 
    );
    Controller.ButtonR1.released(
        []()
        {
            ClawMotor.stop();
        } 
    );
    Controller.ButtonR2.pressed( 
        []() 
        {
            ClawMotor.spin( vex::directionType::rev, 50, vex::percentUnits::pct );
        }
    );
    Controller.ButtonR2.released(
        []()
        {
            ClawMotor.stop();
        } 
    );

    vex::thread calcAngleThread = thread( calcAngle );
    vex::thread moveArmThread = thread( moveArm );
    vex::thread driveBaseThread = thread( driveBase );

    // Main Loop
    while (true)
    {

        this_thread::sleep_for( 15 );
    }
}

float clamp ( float value, float min, float max )
{
    return (value < min) ? min : (value > max) ? max : value;
}

void calcAngles (float &angle1, float &angle2) 
{
    /*
        cosAng2 = (dx^2 + dy^2 - L1^2 - L2^2) / (2 * L1 * L2)
        cosAng2 = clamp(cosAng2, -1, 1) // to avoid NaN due to floating point errors
        ang2 = acos(cosAng2)

        cosAng1 = (dx^2 + dy^2 + L1^2 - L2^2) / (2 * L1 * sqrt(dx^2 + dy^2))
        cosAng1 = clamp(cosAng1, -1, 1) // to avoid NaN due to floating point errors
        ang1 = atan2(dy, dx) - acos(cosAng1)
    */

    float dx2 = TargetX * TargetX;
    float dy2 = TargetY * TargetY;
    float L1_2 = L1 * L1;
    float L2_2 = L2 * L2;
    float dist = sqrt(dx2 + dy2);
    
    float cosAng2 = (dx2 + dy2 - L1_2 - L2_2) / (2 * L1 * L2);
    cosAng2 = clamp(cosAng2, -1.0f, 1.0f); // to avoid NaN due to floating point errors
    angle2 = acos(cosAng2) * (180.0f / M_PI); // Convert to degrees

    float cosAng1 = (dx2 + dy2 + L1_2 - L2_2) / (2 * L1 * dist);
    cosAng1 = clamp(cosAng1, -1.0f, 1.0f); // to avoid NaN due to floating point errors
    angle1 = atan2(TargetY, TargetX) - acos(cosAng1);
    angle1 = angle1 * (180.0f / M_PI); // Convert to degrees
}
