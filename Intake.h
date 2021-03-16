#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>

frc::DoubleSolenoid intakeSolenoid{2, 3};

class Intake
{
private:
    frc::DigitalInput breakBeamFull{8};
    frc::DigitalInput breakBeamNewBall{9};
    frc::DigitalInput breakBeamFifthBall{1};

    bool intakeFull = false;
    bool newBall = false;
    bool fiveBalls = false;

    bool solenoidUp = true;

    // Driver's Left Trigger
    bool isIntaking = false;
    // Driver's Left Bumper
    bool isCycling = false;
    // Operator's Left Trigger
    bool isPurging = false;
    // Operator's Right Trigger
    bool isReversingV = false;
    // Operator's A Temporarily
    bool isTogglingIntake;
    bool firstTogglePress = true;

    // 0.6
    float indexMotorDefault = 0.6;
    // 0.7
    float vMotorDefault = 0.7;

public:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    ctre::phoenix::motorcontrol::can::VictorSPX indexMotor{3};
    ctre::phoenix::motorcontrol::can::VictorSPX conveyorMotor{4};
    ctre::phoenix::motorcontrol::can::TalonFX intakeMotor{5};

    // Run in Init
    void Initiate();
    // Run in Periodic
    void updateButtons();

    void toggleIntakePosition();
    void Run();

    void intake();
    void cycleBalls();
    // Runs intake system in reverse
    void purgeSystem();
    void reverseIndex();
    void shutDown();

    void intakeLogic();
};