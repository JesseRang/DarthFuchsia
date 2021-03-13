#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "Intake.h"

// Use this in robot init // Delete unused solenoid
void Intake::motorSetup()
{
    toggleIntakePosition(solenoidUp);
    intakeMotor.ConfigContinuousCurrentLimit(5, 0);
    intakeFollower.Follow(intakeMotor);

    conveyorMotor.ConfigContinuousCurrentLimit(5, 0);
    conveyorMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    indexMotor.SetSmartCurrentLimit(5);
    indexMotor.SetInverted(true);
}

void Intake::updateButtons()
{
    isIntaking = driverController.GetRawButton(7);
    isCycling = driverController.GetRawButton(5);
    isPurging = operatorController.GetRawButton(7);
    isReversingV = operatorController.GetRawButton(8);
}

void Intake::toggleIntakePosition(bool solenoidUp)
{
    // Delete unused solenoid
    if (solenoidUp)
    {
        //intakeSolenoid.Set(false);
        //intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        solenoidUp = false;
    } 
    else 
    {
        //intakeSolenoid.Set(true);
        //intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        solenoidUp = true;
    }

}

void Intake::Run()
{
    updateButtons();
    if (isIntaking)
        intake();
    else if (isCycling)
        cycleBalls();
    else if (isPurging)
        purgeSystem();
    else if (isReversingV)
        reverseV();
    else
        shutDown();
}

void Intake::intake()
{
    Intake::intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, indexMotorDefault);
    vMotor1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, vMotorDefault);
    vMotor2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, vMotorDefault + 0.2);
    intakeLogic();
}
void Intake::cycleBalls()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    vMotor1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, vMotorDefault);
    vMotor2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, vMotorDefault + 0.2);
    intakeLogic();
}
void Intake::purgeSystem()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    indexMotor.Set(-1);
}
void Intake::reverseV()
{
    vMotor1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    vMotor2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
}
void Intake::shutDown()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    vMotor1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    vMotor2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    indexMotor.Set(0);
    conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}

void Intake::intakeLogic()
{
    intakeFull = !breakBeamFull.Get();
    newBall = !breakBeamNewBall.Get();
    fourBalls = !breakBeamFourthBall.Get();
    fiveBalls = !breakBeamFifthBall.Get();

    if (intakeFull && newBall && fourBalls && fiveBalls)
        indexMotor.Set(0);
    else
        indexMotor.Set(0.5);

    if (intakeFull)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    else if (newBall)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
    else
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}
