// Copyright (c) 2013 Dawn Robotics Ltd - Alan Broun <abroun@dawnrobotics.co.uk>

#include <stdint.h>
#include <Servo.h>
#include "ultrasonic_sensor.h"
#include "rover_motor.h"
#include "rover_ir_sensors.h"

enum eRobotState
{
    eRS_Invalid = -1,
    eRS_DrivingForwardsLookingForWall,
    eRS_BackingUp,
    eRS_EdgingForward,
    eRS_TurningLeft,
    eRS_FollowingWallOnRight
};

const int NUM_IR_SENSORS = 4;
const int IR_LED_PINS[ NUM_IR_SENSORS ] = { A0, A0, A1, A1 };   
const int IR_SENSOR_PINS[ NUM_IR_SENSORS ] = { A3, A2, A4, A5 };

const int LEFT_DIR_PIN = 12;
const int LEFT_PWM_PIN = 11;
const int LEFT_ENCODER_FIRST_PIN = 3;
const int LEFT_ENCODER_SECOND_PIN = 5;
const int LEFT_CURRENT_PIN = A6;

const int RIGHT_DIR_PIN = 7;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;
const int RIGHT_CURRENT_PIN = A7;

const int PAN_SERVO_PIN = 9;
const int PAN_TILT_PIN = 10;
const int ULTRASONIC_SENSOR_PIN = 8;

const float DRIVE_FOWARD_RPM = 50.0;
const float BACKING_UP_RPM = -30.0;
const float EDGING_FORWARD_RPM = 30.0;
const float ABS_TURNING_RPM = 40.0;
const float BASE_WALL_FOLLOWING_RPM = 60.0;

const float CLOSE_ULTRASONIC_RANGE = 0.1;
const int CLOSE_RANGE_IR_VALUE = 150;

const int LOOK_FORWARD_PAN_ANGLE = 90;
const int LOOK_FORWARD_TILT_ANGLE = 90;
const int LOOK_RIGHT_PAN_ANGLE = 0;
const int LOOK_RIGHT_TILT_ANGLE = 70;

const int NUM_BACKING_UP_TICKS = 250;
const int NUM_EDGING_FORWARD_TICKS = 250;
const int NUM_TURN_TICKS = 450;
const int MAX_TURN_TIME_MS = 3000;

const float WALL_FAR_DISTANCE = 0.25f;
const float WALL_CLOSE_DISTANCE = 0.24f;

RoverMotor gLeftMotor( LEFT_DIR_PIN, LEFT_PWM_PIN,
    LEFT_ENCODER_FIRST_PIN, LEFT_ENCODER_SECOND_PIN, LEFT_CURRENT_PIN );
RoverMotor gRightMotor( RIGHT_DIR_PIN, RIGHT_PWM_PIN,
    RIGHT_ENCODER_FIRST_PIN, RIGHT_ENCODER_SECOND_PIN, RIGHT_CURRENT_PIN );

UltrasonicSensor gUltrasonicSensor( ULTRASONIC_SENSOR_PIN );
RoverIRSensors gRoverIRSensors( 
    IR_LED_PINS[ 0 ], IR_LED_PINS[ 1 ],
    IR_LED_PINS[ 2 ], IR_LED_PINS[ 3 ],
    IR_SENSOR_PINS[ 0 ], IR_SENSOR_PINS[ 1 ],
    IR_SENSOR_PINS[ 2 ], IR_SENSOR_PINS[ 3 ] );

Servo gPanServo;
Servo gTiltServo;
int gPanAngle = LOOK_FORWARD_PAN_ANGLE;
int gTiltAngle = LOOK_FORWARD_TILT_ANGLE;
eRobotState gRobotState = eRS_Invalid;
long gStateStartEncoderTicks = 0;
unsigned long gStateStartTimeMS = 0;

//------------------------------------------------------------------------------
void enterDrivingForwardsLookingForWallState();
void enterBackingUpState();
void enterEdgingForwardState();
void enterTurningLeftState();
void enterFollowingWallOnRightState();

//------------------------------------------------------------------------------
void setup()
{
    gPanServo.attach( PAN_SERVO_PIN );
    gTiltServo.attach( PAN_TILT_PIN );
    
    // Start driving forwards
    enterDrivingForwardsLookingForWallState();
    
    Serial.begin( 9600 );
}

//------------------------------------------------------------------------------
void loop()
{
    // Read from the robot's sensors
    float ultrasonicRange = gUltrasonicSensor.measureRange();
    gRoverIRSensors.takeReadings();
    int frontLeftIR = gRoverIRSensors.lastFrontLeftReading();
    int frontRightIR = gRoverIRSensors.lastFrontRightReading();
    int rearLeftIR = gRoverIRSensors.lastRearLeftReading();
    int rearRightIR = gRoverIRSensors.lastRearRightReading();

    gLeftMotor.update();
    gRightMotor.update();

    // Update the state of the robot
    unsigned long curTimeMS = millis();
    
    switch ( gRobotState )
    {
    case eRS_DrivingForwardsLookingForWall:
        {
            // Check to see if we've hit an obstacle we didn't see
            if ( gLeftMotor.isStalled()
                || gRightMotor.isStalled() )
            {
                enterBackingUpState();
            }
            else
            {                
                // Check to see if we've found a wall
                if ( ultrasonicRange <= CLOSE_ULTRASONIC_RANGE
                    || frontLeftIR >= CLOSE_RANGE_IR_VALUE
                    || frontRightIR >= CLOSE_RANGE_IR_VALUE )
                {
                    enterTurningLeftState();
                }
            }

            break;
        }
    case eRS_BackingUp:
        {
            if ( gLeftMotor.isStalled()
                || gRightMotor.isStalled()
                || rearLeftIR >= CLOSE_RANGE_IR_VALUE
                || rearRightIR >= CLOSE_RANGE_IR_VALUE )
            {
                enterEdgingForwardState();
            }
            else if ( abs( gStateStartEncoderTicks - gLeftMotor.getLastMeasuredEncoderTicks() ) >= NUM_BACKING_UP_TICKS )
            {
                enterTurningLeftState();
            }
            break;
        }
    case eRS_EdgingForward:
        {
            if ( gLeftMotor.isStalled()
                || gRightMotor.isStalled()
                || frontLeftIR >= CLOSE_RANGE_IR_VALUE
                || frontRightIR >= CLOSE_RANGE_IR_VALUE
                || abs( gStateStartEncoderTicks - gLeftMotor.getLastMeasuredEncoderTicks() ) >= NUM_EDGING_FORWARD_TICKS )
            {
                enterTurningLeftState();
            }
            break;
        }
    case eRS_TurningLeft:
        {
            if ( abs( gStateStartEncoderTicks - gLeftMotor.getLastMeasuredEncoderTicks() ) >= NUM_TURN_TICKS
               || curTimeMS - gStateStartTimeMS > MAX_TURN_TIME_MS )
            {
                enterFollowingWallOnRightState();
            }
            break;
        }
    case eRS_FollowingWallOnRight:
        {
            // Check to see if we've hit an obstacle we didn't see
            if ( gLeftMotor.isStalled()
                || gRightMotor.isStalled() )
            {
                enterBackingUpState();
            }
            else
            {                
                // Check to see if we've found a wall
                if ( frontLeftIR >= CLOSE_RANGE_IR_VALUE
                    || frontRightIR >= CLOSE_RANGE_IR_VALUE
                    || rearLeftIR >= CLOSE_RANGE_IR_VALUE
                    || rearRightIR >= CLOSE_RANGE_IR_VALUE )
                {
                    enterBackingUpState();
                }
                else
                {
                    if ( ultrasonicRange > WALL_FAR_DISTANCE )
                    {
                        gLeftMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM + 20.0 );
                        gRightMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM - 20.0 );
                    }
                    else if ( ultrasonicRange < WALL_CLOSE_DISTANCE )
                    {
                        gLeftMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM - 20.0 );
                        gRightMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM + 20.0 );
                    }
                    else
                    {
                        gLeftMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM );
                        gRightMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM );
                    }
                }
            }
            
            break;
        }
    default:
        {
            // We should never get here, but return to the 
            // first state if we do
            enterDrivingForwardsLookingForWallState();
        }
    }    
    
    gPanServo.write( gPanAngle );
    gTiltServo.write( gTiltAngle );
    
    // Output debug info here
    Serial.print( gLeftMotor.getLastMeasuredRPM() );
    Serial.print( " " );
    Serial.print( gRightMotor.getLastMeasuredRPM() );
    Serial.print( " " );
    Serial.print( gLeftMotor.getCurDutyCycle() );
    Serial.print( " " );
    Serial.println( gRightMotor.getCurDutyCycle() );
}

//------------------------------------------------------------------------------
void enterDrivingForwardsLookingForWallState()
{
    // Point the ultrasonic sensor forward
    gPanAngle = LOOK_FORWARD_PAN_ANGLE;
    gTiltAngle = LOOK_FORWARD_TILT_ANGLE;

    gPanServo.write( gPanAngle );
    gTiltServo.write( gTiltAngle );
    delay( 1000 );

    gLeftMotor.clearStall();
    gRightMotor.clearStall();

    gLeftMotor.setTargetRPM( DRIVE_FOWARD_RPM );
    gRightMotor.setTargetRPM( DRIVE_FOWARD_RPM );

    gStateStartEncoderTicks = gLeftMotor.getLastMeasuredEncoderTicks();
    gRobotState = eRS_DrivingForwardsLookingForWall;
}

//------------------------------------------------------------------------------
void enterBackingUpState()
{
    gLeftMotor.clearStall();
    gRightMotor.clearStall();

    gLeftMotor.setTargetRPM( BACKING_UP_RPM );
    gRightMotor.setTargetRPM( BACKING_UP_RPM );

    gStateStartEncoderTicks = gLeftMotor.getLastMeasuredEncoderTicks();
    gRobotState = eRS_BackingUp;
}

//------------------------------------------------------------------------------
void enterEdgingForwardState()
{
    gLeftMotor.clearStall();
    gRightMotor.clearStall();

    gLeftMotor.setTargetRPM( EDGING_FORWARD_RPM );
    gRightMotor.setTargetRPM( EDGING_FORWARD_RPM );

    gStateStartEncoderTicks = gLeftMotor.getLastMeasuredEncoderTicks();
    gStateStartTimeMS = millis();
    gRobotState = eRS_EdgingForward;
}

//------------------------------------------------------------------------------
void enterTurningLeftState()
{
    gLeftMotor.clearStall();
    gRightMotor.clearStall();

    gLeftMotor.setTargetRPM( -ABS_TURNING_RPM );
    gRightMotor.setTargetRPM( ABS_TURNING_RPM );

    gStateStartEncoderTicks = gLeftMotor.getLastMeasuredEncoderTicks();
    gStateStartTimeMS = millis();
    gRobotState = eRS_TurningLeft;
}

//------------------------------------------------------------------------------
void enterFollowingWallOnRightState()
{
    // Point the ultrasonic sensor to the right
    gPanAngle = LOOK_RIGHT_PAN_ANGLE;
    gTiltAngle = LOOK_RIGHT_TILT_ANGLE;

    gPanServo.write( gPanAngle );
    gTiltServo.write( gTiltAngle );

    gLeftMotor.clearStall();
    gRightMotor.clearStall();

    gLeftMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM );
    gRightMotor.setTargetRPM( BASE_WALL_FOLLOWING_RPM );

    gStateStartEncoderTicks = gLeftMotor.getLastMeasuredEncoderTicks();
    gStateStartTimeMS = millis();
    gRobotState = eRS_FollowingWallOnRight;
}


