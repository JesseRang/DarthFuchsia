#include "Shooter.h"
#include "Robot.h"
#include "Intake.h" //added to test four break beam logic
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
void Shooter::Initiate()
{
    shooterMotorR.Follow(shooterMotorL);

    shooterMotorL.ConfigFactoryDefault();
    //shooterMotorL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
    //shooterMotorR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
    shooterMotorL.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, timeoutMS);
    shooterMotorL.SetSensorPhase(true);

    shooterMotorR.SetInverted(true);
    shooterMotorL.SetInverted(false);
    shooterMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    shooterMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    shooterMotorL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 50, 0.01));
    shooterMotorR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 50, 0.01));

    setPIDFValues(false);
}

// Run in Periodic
void Shooter::updateButtons()
{
    trenchButtonPressed = operatorController.GetRawButton(4);
    initButtonPressed = driverController.GetRawButton(6); //changed from operator controller for scoring challenge also should be raw btn 3
    wallButtonPressed = operatorController.GetRawButton(1);
    testButtonPressed = operatorController.GetRawButton(2);

    isShooting = driverController.GetRawAxis(3);

    isLimelightActive = driverController.GetRawButton(1);

    if (!trenchButtonPressed && !initButtonPressed && !wallButtonPressed && !isLimelightActive)
    {
        counter = 0;
    }
}

void Shooter::updateLimelight(double ty, double hasTarget)
{
    mLimeLight.targetPositionY = ty;
    mLimeLight.limelightHasTarget = hasTarget;
}

void Shooter::Shoot()
{
    if (isShooting > 0.05)
    {
        activateConveyor();
    }

    if (trenchButtonPressed || initButtonPressed || wallButtonPressed)
    {
        modifyWheelVelocity();
    }
    else if ((isLimelightActive && !trenchButtonPressed && !initButtonPressed && !wallButtonPressed) || autoShot == true)
    {
        setLimelightSpeed();
    }
    else
    {
        flyWheelDesiredSpeed = 0;
        shooterMotorL.Set(TalonFXControlMode::PercentOutput, 0);
        setHoodPosition(-0.58);
    }
}

void Shooter::printShooterSpeeds()
{
    //frc::SmartDashboard::PutNumber("Trench Speed", 5500); //6000
    //frc::SmartDashboard::PutNumber("Init Speed", 5500);
    //frc::SmartDashboard::PutNumber("Wall Speed", 2700);
    //frc::SmartDashboard::PutNumber("limelightSpeed", limelightSpeed);
}

void Shooter::getShooterSpeeds()
{
    //trenchSpeed = frc::SmartDashboard::GetNumber("Trench Speed", 5500);
    //initSpeed = frc::SmartDashboard::GetNumber("Init Speed", 5500);
    //wallSpeed = frc::SmartDashboard::GetNumber("Wall Speed", 2700);

    setLimelightSpeed();
}

void Shooter::printPIDFValues()
{
    //frc::SmartDashboard::PutNumber("ShooterF", shooterF);
    //frc::SmartDashboard::PutNumber("ShooterP", shooterP);
    //frc::SmartDashboard::PutNumber("ShooterI", shooterI);
    //frc::SmartDashboard::PutNumber("ShooterD", shooterD);
    //frc::SmartDashboard::PutNumber("WallP", wallP);

    frc::SmartDashboard::PutNumber("ShooterVelocity", shooterMotorL.GetSelectedSensorVelocity() / 3.4133);
    //frc::SmartDashboard::PutNumber("Command Velocity", commandShooter); //added to adjust desired velocity from smart dashboard
    //frc::SmartDashboard::PutNumber("Hood Position", hoodPosition); //added to adjust hood angle from smart dashboard
}

void Shooter::setPIDFValues(bool isWallShot)
{
    shooterMotorL.Config_kF(0, shooterF, timeoutMS);
    //shooterMotorL.Config_kI(0, shooterI, timeoutMS); //commented out to try to fix bugs
    shooterMotorL.Config_kD(0, shooterD, timeoutMS);

    shooterP = frc::SmartDashboard::GetNumber("ShooterP", shooterP);
    shooterMotorL.Config_kP(0, shooterP, timeoutMS);

    /*if (isWallShot)
        //shooterMotorL.Config_kP(0, wallP, timeoutMS); //commented out to adjust pid from smart dashboard
        if (wallP != frc::SmartDashboard::GetNumber("WallP", 0))
        {
            wallP = frc::SmartDashboard::GetNumber("WallP", 0);
            shooterMotorL.Config_kP(0, wallP, timeoutMS);
        }
    else
    {
        if (shooterP != frc::SmartDashboard::GetNumber("ShooterP", shooterP))
        {
            shooterP = frc::SmartDashboard::GetNumber("ShooterP", shooterP);
            shooterMotorL.Config_kP(0, shooterP, timeoutMS);
        }
    }*/

    /*if (shooterI != frc::SmartDashboard::GetNumber("ShooterI", 0))
    {
        shooterI = frc::SmartDashboard::GetNumber("ShooterI", 0);
        //shooterMotorL.Config_kI(0, shooterI, timeoutMS);
    }

    if (shooterD != frc::SmartDashboard::GetNumber("ShooterD", 1.5))
    {
        shooterD = frc::SmartDashboard::GetNumber("ShooterD", 1.5);
        shooterMotorL.Config_kD(0, shooterD, timeoutMS);
    }

    if (shooterF != frc::SmartDashboard::GetNumber("ShooterF", 0.047))
    {
        shooterF = frc::SmartDashboard::GetNumber("ShooterF", 0.047);
        shooterMotorL.Config_kF(0, shooterF, timeoutMS);
    }

    if (hoodPosition != frc::SmartDashboard::GetNumber("Hood Postion", hoodPosition))
    {
        hoodPosition = frc::SmartDashboard::GetNumber("Hood Position", hoodPosition);
        setHoodPosition(frc::SmartDashboard::GetNumber("Hood Position", hoodPosition));
    }*/
}

bool autoShot = false;

void Shooter::setLimelightSpeed()
{
    double modValue = 3.4133;
    double limelightDistanceOffset = 4;

    if (isLimelightActive || autoShot == true)
    {
        //std::cout << "limlightActive" << std::endl;
        mLimeLight.ledMode = 3;
        if (mLimeLight.limelightHasTarget)
        {

            if (!powerPortChallenge)
            {
                flyWheelDesiredSpeed = limelightSpeed * modValue;
                setHoodPosition((0.0181 * (mLimeLight.ty - limelightDistanceOffset)) - 0.57); //added -limelightDistanceOffset for more accuracy with 3d goal` original value -0.548
            }
            else
            {
                shooterP = 1.6;    //1.6
                shooterD = 3;      //3
                shooterF = 0.0475; //0.0475

                setHoodPosition(-0.43 + (0.0382 * mLimeLight.ty) + (0.00146 * (mLimeLight.ty * mLimeLight.ty))); //-0.65

                flyWheelDesiredSpeed = (2400 + (-53.2 * mLimeLight.ty) + (-1.22 * (mLimeLight.ty * mLimeLight.ty))) * modValue; //2650
                if (flyWheelDesiredSpeed > 3000 * modValue)
                {
                    flyWheelDesiredSpeed = 3000 * modValue;
                }
            }
            shooterMotorL.Set(TalonFXControlMode::Velocity, flyWheelDesiredSpeed);
        }
    }
    else if (!trenchButtonPressed && !initButtonPressed && !wallButtonPressed)
    {
        shooterMotorL.Set(TalonFXControlMode::PercentOutput, 0);
    }

    //std::cout << "limelight hood " << 0.0181 * mLimeLight.ty - 0.548 << std::endl;
}

void Shooter::activateConveyor()
{
    shooterConveyor.indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7); //used to be 0.7
    if (trenchButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5); //used to be 1 //changed from 0.5 to 0.25 bc we switched to a falcon
    }
    else if (initButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7); //0.7
    }
    else if (wallButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5); //0.5
    }
    else if (isLimelightActive || autoShot == true)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1); //0.7
    }

    //std::cout << "counter" << counter << std::endl;
}

void Shooter::modifyWheelVelocity()
{
    double modValue = 3.4133;
    float trenchHoodPosition = -1; //changed from -0.3
    float initHoodPosition = 0;    //changed from 0.7
    float wallHoodPosition = 1;
    float defaultHoodPosition = -0.6;
    //float getHoodPosition = hoodServo.GetSpeed();

    if (trenchButtonPressed)
    {
        flyWheelDesiredSpeed = trenchSpeed * modValue;
        setHoodPosition(trenchHoodPosition);
    }
    else if (initButtonPressed)
    {
        flyWheelDesiredSpeed = initSpeed * modValue;
        setHoodPosition(initHoodPosition);
    }
    else if (wallButtonPressed)
    {
        //flyWheelDesiredSpeed = wallSpeed * modValue;
        setHoodPosition(wallHoodPosition);
    }

    if (flyWheelDesiredSpeed == 0)
    {
        shooterMotorL.Set(TalonFXControlMode::PercentOutput, 0);
    }
    else
        shooterMotorL.Set(TalonFXControlMode::Velocity, flyWheelDesiredSpeed);
}

void Shooter::setHoodPosition(float position)
{
    // 0 - 1. 0 fully extended and 1 is fully retracted
    hoodServo.SetBounds(1.75, 1.7, 1.5, 1.2, 1.1);
    //hoodServo.SetRawBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    hoodServo.SetSpeed(position);
}

void Shooter::Stop()
{
    shooterMotorL.Set(TalonFXControlMode::PercentOutput, 0);
}