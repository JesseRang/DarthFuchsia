#pragma once

#include "Intake.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Servo.h>

class Shooter
{
private:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    ctre::phoenix::motorcontrol::can::TalonFX shooterMotorL{0};
    ctre::phoenix::motorcontrol::can::TalonFX shooterMotorR{1};

    double shooterF = 0.0453;
    double shooterP = 0.15;
    double shooterI = 0;
    double shooterD = 1.5;
    double wallP = 0.125;

    // Operator Y
    bool trenchButtonPressed = false;
    // Operator X
    bool initButtonPressed = false;
    // Operator A
    bool wallButtonPressed = false;
    // Driver A
    bool isLimelightActive = false;
    // Driver Right Trigger
    bool isShooting = false;

    double trenchSpeed = 6000;
    double initSpeed = 5500;
    double wallSpeed = 2700;
    double limelightSpeed = 0;

    double flyWheelDesiredSpeed = 0;

    bool limelightHasTarget = false;

    int timeoutMS = 30;

    frc::Servo hoodServo{1};

    Intake shooterConveyor;

public:
    // Run in Init
    void Initiate();
    // Run in Periodic
    void updateButtons();

    void Shoot();

    void printShooterSpeeds();
    void getShooterSpeeds();

    // Displays PIDF Values on Smart Dashboard
    void printPIDFValues();
    // Gets the value of Smart Dashboard
    void getPIDFValues();
    // Sets motors to PIDF previously gotten
    void setPIDFValues(bool isWallShot);

    void setLimelightSpeed();

    void activateConveyor();
    void modifyWheelVelocity();

    void setHoodPosition(float position);
};