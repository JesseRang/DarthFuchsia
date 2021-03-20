#pragma once
#ifndef INTAKE_H_
#define INTAKE_H_

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>

//frc::DoubleSolenoid intakeSolenoid{2, 3};

class Intake
{
private:
    bool index = false;
    bool newBall = false;
    bool intakeFull = false;
    bool conveyorStart = false;

    bool solenoidUp = true;

    // Driver's Left Trigger
    float isIntaking = 0;
    // Driver's Left Bumper
    bool isCycling = false;
    // Operator's Left Trigger
    bool isPurging = false;
    // Operator's Right Trigger
    bool isReversingV = false;
    // Operator's A Temporarily
    bool isTogglingIntake;
    bool firstTogglePress = true;

    bool stillNewBall;

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
    void updateBreakBeams(bool newBallBreakBeambool, bool indexBreakBeam, bool conveyorStartBreakBeam, bool intakeFullBreakBeam);
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

#endif