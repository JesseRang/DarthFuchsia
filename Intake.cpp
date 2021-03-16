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

void Intake::updateButtons()
{
    isIntaking = driverController.GetRawAxis(2);
    isCycling = driverController.GetRawButton(5);
    isPurging = operatorController.GetRawAxis(2);
    isReversingV = operatorController.GetRawButton(8);
    isTogglingIntake = driverController.GetRawButton(1);
}

void Intake::toggleIntakePosition()
{
    /*
    if (isTogglingIntake)
    {
        //intakeSolenoid.Toggle();
        /*  if (solenoidUp && firstTogglePress)
        {
            intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
            solenoidUp = false;
            std::printf("Lowering Solenoid\n");
        }
        else if (!solenoidUp && firstTogglePress)
        {
            std::printf("Raising Solenoid\n");
            intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
            solenoidUp = true;
        }
        firstTogglePress = false;
    }
    else
    {
        std::printf("reset");
        firstTogglePress = true;
    } */
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
        reverseIndex();
    else
        shutDown();
}

void Intake::intake()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.6);
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
    intakeFull = !breakBeamFull.Get();
    newBall = !breakBeamNewBall.Get();
    fiveBalls = !breakBeamFifthBall.Get();
   /*  std::printf("New Ball\b\n", newBall);
    std::printf("Intake Full\b\n", intakeFull); */
    std::cout << "Intake Full\n" << intakeFull << " " << !breakBeamFull.Get();

    /* if (intakeFull && newBall && fiveBalls)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

    }
    else
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);

    if (intakeFull)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    else if (newBall)
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
    else
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
 */}
