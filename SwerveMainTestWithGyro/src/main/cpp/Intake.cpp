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

void Intake::updateBreakBeams(bool newBallBreakBeam, bool indexBreakBeam, bool conveyorStartBreakBeam, bool intakeFullBreakBeam, bool conveyorSpaceBreakBeam)
{
    newBall = !newBallBreakBeam;
    index = !indexBreakBeam;
    conveyorStart = !conveyorStartBreakBeam;
    intakeFull = !intakeFullBreakBeam;
    conveyorSpace = !conveyorSpaceBreakBeam;
}

bool fourBalls = 0;
bool fourthDone = 0;

void Intake::updateButtons()
{
    isIntaking = driverController.GetRawAxis(2);
    isCycling = driverController.GetRawButton(5);
    isPurging = operatorController.GetRawAxis(2);
    isReversingV = operatorController.GetRawButton(8);
    isTogglingIntake = operatorController.GetRawButton(6);
    double isShooting = driverController.GetRawAxis(3);
    if (isShooting > 0.05)
    {
        fourBalls = 0;
        fourthDone = 0;
    }
}

void Intake::Run()
{
    updateButtons();
    if (isIntaking > 0 || autonomousIntake == true)
        intake();
    else if (isCycling)
        cycleBalls();
    else if (isPurging)
    {
        purgeSystem();
        fourthDone = 0;
        fourBalls = 0;
    }
    else if (isReversingV)
        reverseIndex();
    else
        shutDown();
}

void Intake::intake()
{
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1); //commented out to save amps for power port challenge
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
    conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4); //-0.5
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

int counter = 0;
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
    /*if (index && intakeFull)
    {
        //indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        //conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        std::cout << "index & intake full" << std::endl;
        if (newBall && index)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
            std::cout << "newBall & index" << std::endl;
        }
        else
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            std::cout << "index motor 0" << std::endl;
        }
    } 
    else if (intakeFull)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7); //changed from 0.7 to 0 to add spacing logic
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        if (newBall && fourBalls == 0)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
            fourBalls = 1;
            counter = 0;
            std::cout << "fourBalls" << std::endl;
        }
        else if (!newBall && fourBalls == 1)
        {
            fourthDone = 1;
            std::cout << "fourth done" << std::endl;
        }
        else if (newBall && fourthDone == 1)
        {
            while (counter <= 10)
            {
                indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
                counter++;
            }
            std::cout << "five balls" << std::endl;
        }
        std::cout << "intakefull" << std::endl;
    }
    else if (!intakeFull && conveyorStart)
    {
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.6); //1
        std::cout << "!intake full & conveyorstart" << std::endl;
    }
    else
    {
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
        std::cout << "else" << std::endl;
    }*/

    //ADDED 4/22, CODE BUILT FOR 4 BREAK BEAMS //
    if (!intakeFull)
    {
        //std::cout << "!intakeFull" << std::endl;

        indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);

        /*if (newBall || index)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
            std::cout << "newBall" << std::endl;
        }
        else
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }*/

        if (conveyorStart || conveyorSpace)
        {
            conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7); //used to be 0.5
            //std::cout << "conveyorStart/conveyorSpace" << std::endl;
        }
        else
        {
            conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }
    }
    else if (intakeFull)
    {
        //std::cout << "intakeFull" << std::endl;
        conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

        if (fourthDone == 0)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.6); //0.7
            //std::cout << "run index" << std::endl;
            //std::cout << "new ball: " << newBall << std::endl;
            //std::cout << "index: " << index <<std::endl;
            //std::cout << "four balls: " << fourBalls << std::endl;
            //std::cout << "fourth done: " << fourthDone << std::endl;
        }

        if (index == 1 && fourBalls == 0)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            fourBalls = 1;
            counter = 0;
            //std::cout << "fourth ball" << std::endl;
        }
        else if ((newBall == 0) && fourBalls == 1)
        {
            fourthDone = 1;
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            //std::cout << "fourth done" << std::endl;
        }

        if ((newBall /*|| index*/) && fourthDone == 1)
        {
            if (counter <= 10)
            {
                indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
                counter++;
            }
            else
            {
                indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            }

            //std::cout << "fifth ball" << std::endl;
        }
        else if (fourthDone == 1)
        {
            indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }
    }
}
