#pragma once

#include "Intake.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Shooter
{
private:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    ctre::phoenix::motorcontrol::can::TalonFX::TalonFX shooterMotorL{0};
    ctre::phoenix::motorcontrol::can::TalonFX::TalonFX shooterMotorR{1};

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
    // Operator B
    bool limelightButtonPressed = false;
    // Driver A
    bool isLimelightActive = false;

    double trenchSpeed = 6000;
    double initSpeed = 5500;
    double wallSpeed = 2700;
    double limelightSpeed = 0;

    bool limelightHasTarget = false;

    int timeoutMS = 30;

    Intake shooterConveyor;

public:
    // Run in Init
    void Initiate();
    // Run in Periodic
    void updateButtons();

    void Shoot();

    void printCurrentSpeeds();
    void printShooterSpeeds();
    void setShooterSpeeds();

    void printPIDFValues();
    void printCurrentPIDFValues();
    void setPIDFValues();

    void setLimelightSpeed();

    void activateConveyor();
    void modifyVelocity();

    void setShooterPIDF(bool isWallShot);
    void setHoodPosition();
};