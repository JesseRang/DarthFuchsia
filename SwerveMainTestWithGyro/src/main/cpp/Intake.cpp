#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "Intake.h"

// Use this in robot init
void Intake::Initiate()
{
    intakeMotor.SetInverted(true);
    //intakeMotor.ConfigContinuousCurrentLimit(5, 0);

    //conveyorMotor.ConfigContinuousCurrentLimit(5, 0);
    conveyorMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    conveyorMotor.SetInverted(true);

    //indexMotor.ConfigContinuousCurrentLimit(5, 0);
}

void Intake::updateBreakBeams(bool newBallBreakBeam, bool indexBreakBeam, bool conveyorStartBreakBeam, bool intakeFullBreakBeam)
{
    newBall = !newBallBreakBeam;
    index = !indexBreakBeam;
    conveyorStart = !conveyorStartBreakBeam;
    intakeFull = !intakeFullBreakBeam;
}

void Intake::updateButtons()
{
    isIntaking = driverController.GetRawAxis(2);
    isCycling = driverController.GetRawButton(5);
    isPurging = operatorController.GetRawAxis(2);
    isReversingV = operatorController.GetRawButton(8);
    isTogglingIntake = operatorController.GetRawButton(6);
}

void Intake::Run()
{
    updateButtons();
    if (isIntaking > 0)
        intake();
    else if (isCycling)
        cycleBalls();
    else if (isPurging)
        purgeSystem();
    else if (isReversingV)
        reverseIndex();
    else
        shutDown();
}

void Intake::intake()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    intakeLogic();
}
void Intake::cycleBalls()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    intakeLogic();
}
void Intake::purgeSystem()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    conveyorMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -0.5);
    indexMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -1);
}
void Intake::reverseIndex()
{
    indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
}
void Intake::shutDown()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}

void Intake::intakeLogic()
{
    /*if (newBall && conveyorStart && intakeFull)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    else if (intakeFull)
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
    else 
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);

    if (intakeFull)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    else if (conveyorStart)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    else
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);*/

    //added 3/19 to set up logic for 4 break beams instead of 3
    if (index && intakeFull)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        if (newBall && index)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
        }
        else
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }
    } 
    else if (intakeFull)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    else if (!intakeFull && conveyorStart)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.8); //1
    }
    else
    {
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    }
}
