#pragma once

#include "Intake.h"
#include "LimeLight.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Servo.h>


class Shooter
{
private:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    ctre::phoenix::motorcontrol::can::TalonFX shooterMotorL{0}; //can id should be 0
    ctre::phoenix::motorcontrol::can::TalonFX shooterMotorR{1}; //can id should be 1

    double shooterF = 0.05;
    double shooterP = 4.75; //4.75
    double shooterI = 0;
    double shooterD = 1; //1
    double wallP = 0.25; //0.125
    double commandShooter = 0;
    float hoodPosition = 0;

    // Operator Y
    bool trenchButtonPressed = false;
    // Operator X
    bool initButtonPressed = false;
    // Operator A
    bool wallButtonPressed = false;
    //Operator B
    bool testButtonPressed = false;
    // Driver A
    bool isLimelightActive = false;
   

    double trenchSpeed = 5500; //6000
    double initSpeed = 5500;
    double wallSpeed = 3000; //2700
    double limelightSpeed = 2880; ///5075

    double flyWheelDesiredSpeed = 0; //should be 0
    bool powerPortChallenge = true;

    int timeoutMS = 30;
    int counter = 0;

    frc::Servo hoodServo{1};

    Intake shooterConveyor;

public:
    LimeLight mLimeLight;
    // Run in Init
    void Initiate();
    // Run in Periodic
    void updateButtons();
    void updateLimelight(double ty, double hasTarget);

    void Shoot();

    void printShooterSpeeds();
    void getShooterSpeeds();

    // Displays PIDF Values on Smart Dashboard
    void printPIDFValues();
    // Gets the value of Smart Dashboard
    void getPIDFValues();
    // Sets motors to PIDF previously gotten
    void setPIDFValues(bool isWallShot);

    void limelightInit();

    void setLimelightSpeed();

    void activateConveyor();
    void modifyWheelVelocity();

    void setHoodPosition(float position);
    void Stop();

    bool autoShot;
     // Driver Right Trigger
    double isShooting = 0.00;
};