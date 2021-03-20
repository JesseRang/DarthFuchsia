#include "Shooter.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

void Shooter::Initiate()
{
    shooterMotorR.Follow(shooterMotorL);

    shooterMotorL.ConfigFactoryDefault();
    shooterMotorL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
    shooterMotorR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
    shooterMotorL.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, timeoutMS);
    shooterMotorL.SetSensorPhase(true);

    shooterMotorR.SetInverted(true);
    shooterMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    shooterMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    setPIDFValues(false);
}

// Run in Periodic
void Shooter::updateButtons()
{
    trenchButtonPressed = operatorController.GetRawButton(4);
    initButtonPressed = operatorController.GetRawButton(3);
    wallButtonPressed = operatorController.GetRawButton(1);

    isShooting = driverController.GetRawAxis(3);

    isLimelightActive = driverController.GetRawButton(2);
}

void Shooter::updateLimelight(double ty, bool hasTarget)
{
    targetPositionY = ty;
    limelightHasTarget = hasTarget;
}

void Shooter::Shoot()
{
    if (isShooting > 0.05)
    {
        getShooterSpeeds();
        activateConveyor();
    }
    modifyWheelVelocity();
}

void Shooter::printShooterSpeeds()
{
    frc::SmartDashboard::PutNumber("Trench Speed", 6000);
    frc::SmartDashboard::PutNumber("Init Speed", 5500);
    frc::SmartDashboard::PutNumber("Wall Speed", 2700);
    frc::SmartDashboard::PutNumber("limelightSpeed", limelightSpeed);
}

void Shooter::getShooterSpeeds()
{
    trenchSpeed = frc::SmartDashboard::GetNumber("Trench Speed", 6000);
    initSpeed = frc::SmartDashboard::GetNumber("Init Speed", 5500);
    wallSpeed = frc::SmartDashboard::GetNumber("Wall Speed", 2700);

    setLimelightSpeed();
}

void Shooter::printPIDFValues()
{
    frc::SmartDashboard::PutNumber("ShooterF", 0.047);
    frc::SmartDashboard::PutNumber("ShooterP", 0.125);
    frc::SmartDashboard::PutNumber("ShooterI", 0);
    frc::SmartDashboard::PutNumber("ShooterD", 1.5);
    frc::SmartDashboard::PutNumber("WallP", 0.125);

    frc::SmartDashboard::PutNumber("ShooterVelocity", shooterMotorL.GetSelectedSensorVelocity() / 3.4133);
}

void Shooter::getPIDFValues()
{
    shooterF = frc::SmartDashboard::GetNumber("ShooterF", 0.047);
    shooterP = frc::SmartDashboard::GetNumber("ShooterP", 0.125);
    shooterI = frc::SmartDashboard::GetNumber("ShooterI", 0);
    shooterD = frc::SmartDashboard::GetNumber("ShooterD", 1.5);
    wallP = frc::SmartDashboard::GetNumber("WallP", 0);
}

void Shooter::setPIDFValues(bool isWallShot)
{
    shooterMotorL.Config_kF(0, shooterF, timeoutMS);
    shooterMotorL.Config_kI(0, shooterI, timeoutMS);
    shooterMotorL.Config_kD(0, shooterD, timeoutMS);

    if (isWallShot)
        shooterMotorL.Config_kP(0, wallP, timeoutMS);
    else
        shooterMotorL.Config_kP(0, shooterP, timeoutMS);
}

void Shooter::setLimelightSpeed()
{
    if (isLimelightActive && limelightHasTarget)
    {
        if (targetPositionY > 0)
            limelightSpeed = 0;

        else if (targetPositionY < 0 && targetPositionY >= -0.49)
            limelightSpeed = 6300;

        else if (targetPositionY < -0.49 && targetPositionY >= -6.17)
            limelightSpeed = 5500;

        else if (targetPositionY < -6.17 && targetPositionY >= -7.87)
            limelightSpeed = 5400;

        else if (targetPositionY < -7.17 && targetPositionY >= -8.86)
            limelightSpeed = 5500;

        else if (targetPositionY < -8.86 && targetPositionY >= -13.55)
            limelightSpeed = 5600;

        else if (targetPositionY < -13.55 && targetPositionY >= -14.28)
            limelightSpeed = 5900;

        else if (targetPositionY < -14.28 && targetPositionY >= 15)
            limelightSpeed = 6300;

        else
            limelightSpeed = 0;
    }
}

void Shooter::activateConveyor()
{
    shooterConveyor.indexMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
    if (trenchButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    }
    else if (initButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
    }
    else if (wallButtonPressed)
    {
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
    }
    else if (isLimelightActive)
        shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
}

void Shooter::modifyWheelVelocity()
{
    double modValue = 3.4133;
    float trenchHoodPosition = -1;
    float initHoodPosition = 0;
    float wallHoodPosition = 1;
    float limelightHoodPosition = 0; //pass in something for this value once shooter testing is figured out
    float defaultHoodPosition = 0;

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
        flyWheelDesiredSpeed = wallSpeed * modValue;
        setHoodPosition(wallHoodPosition);
    }
    else if (isLimelightActive)
    {
        flyWheelDesiredSpeed = limelightSpeed * modValue;
        setHoodPosition(limelightHoodPosition);
    }
    else
    {
        flyWheelDesiredSpeed = 0;
        setHoodPosition(defaultHoodPosition);
    }

    if (flyWheelDesiredSpeed == 0)
        shooterMotorL.Set(TalonFXControlMode::PercentOutput, 0);
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