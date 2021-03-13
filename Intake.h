#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>

class Intake
{
private:
    frc::Joystick driverController{0};
    frc::Joystick operatorController{1};

    // Delete Unused Solenoid (search for the beginning of this comment for easy location)
    //frc::Solenoid intakeSolenoid{2};
    //frc::DoubleSolenoid intakeSolenoid{2, 3};

    frc::DigitalInput breakBeamFull{3};
    frc::DigitalInput breakBeamNewBall{2};
    frc::DigitalInput breakBeamFourthBall{1};
    frc::DigitalInput breakBeamFifthBall{0};

    ctre::phoenix::motorcontrol::can::VictorSPX vMotor1{0};
    ctre::phoenix::motorcontrol::can::VictorSPX vMotor2{1};
    ctre::phoenix::motorcontrol::can::VictorSPX intakeFollower{2};

    bool intakeFull = false;
    bool newBall = false;
    bool fourBalls = false;
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

    // 0.6
    float indexMotorDefault = 0.6;
    // 0.7
    float vMotorDefault = 0.7;
    

public:

    rev::CANSparkMax indexMotor{5, rev::CANSparkMax::MotorType::kBrushless};
    ctre::phoenix::motorcontrol::can::TalonSRX intakeMotor{5};
    ctre::phoenix::motorcontrol::can::TalonSRX conveyorMotor{4};
    // Run in Init
    void motorSetup();
    // Run in Periodic
    void updateButtons();

    void toggleIntakePosition(bool solenoidUp);
    void Run(); 
   
    void intake();
    void cycleBalls();
    void purgeSystem();
    // Reverses mechanism known as the V
    void reverseV();
    void shutDown();

    void intakeLogic();
};