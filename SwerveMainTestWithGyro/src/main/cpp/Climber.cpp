#include "Climber.h"

void Climber::Initiate()
{
    climbL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
    climbR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);

    climbR.SetInverted(true);
    
}

void Climber::updateButtons()
{
    climbJoystick = operatorController.GetRawAxis(5);
}

void Climber::controlClimber()
{
    climbL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, climbJoystick);
    climbR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, climbJoystick);
}