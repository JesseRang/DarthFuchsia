#pragma once
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>

class Climber
{
    private:
    double climbJoystick; //operator right y axis
    bool climberBrakeButton; //operator left bumper

    public:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    ctre::phoenix::motorcontrol::can::TalonFX climbL{2}; //can ID not accurate
    ctre::phoenix::motorcontrol::can::TalonFX climbR{3}; //can ID not accurate

    void Initiate();
    void updateButtons();
    void controlClimber();
};