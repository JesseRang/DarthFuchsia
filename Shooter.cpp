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

    shooterMotorL.SetInverted(true);
    shooterMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    shooterMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    setShooterPIDF(false);
}

// Run in Periodic
void Shooter::updateButtons()
{
    trenchButtonPressed = operatorController.GetRawButton(4);
    initButtonPressed = operatorController.GetRawButton(1);
    wallButtonPressed = operatorController.GetRawButton(2);
    limelightButtonPressed = operatorController.GetRawButton(3);
    
    isLimelightActive = driverController.GetRawButton(2);
}

// Connect to shoot button in Robot.cpp
void Shooter::Shoot()
{
    modifyVelocity();
    activateConveyor();
}

void Shooter::printShooterSpeeds()
{
    frc::SmartDashboard::PutNumber("Trench Speed", 6000);
    frc::SmartDashboard::PutNumber("Init Speed", 5500);
    frc::SmartDashboard::PutNumber("Wall Speed", 2700);
    frc::SmartDashboard::PutNumber("limelightSpeed", limelightSpeed);
}

void Shooter::setShooterSpeeds()
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

void Shooter::setPIDFValues()
{
    shooterF = frc::SmartDashboard::GetNumber("ShooterF", 0.047);
    shooterP = frc::SmartDashboard::GetNumber("ShooterP", 0.125);
    shooterI = frc::SmartDashboard::GetNumber("ShooterI", 0);
    shooterD = frc::SmartDashboard::GetNumber("ShooterD", 1.5);
    wallP = frc::SmartDashboard::GetNumber("WallP", 0);
}

void Shooter::setShooterPIDF(bool isWallShot)
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
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double ty = table->GetNumber("ty", 0.0);
    double tv = table->GetNumber("tv", 0.0);

    if (isLimelightActive)
    {
        if (tv < 1.0)
        {
            limelightHasTarget = false;
        }
        else
        {
    
            limelightHasTarget = true;
        }
        if (limelightHasTarget)
        {
            if (ty > 0)
                limelightSpeed = 0;
            
            else if (ty < 0 && ty >= -0.49)
                limelightSpeed = 6300;

            else if (ty < -0.49 && ty >= -6.17)
                limelightSpeed = 5500;
            
            else if (ty < -6.17 && ty >= -7.87)
                limelightSpeed = 5400;
            
            else if (ty < -7.17 && ty >= -8.86)
                limelightSpeed = 5500;
            
            else if (ty < -8.86 && ty >= -13.55)
                limelightSpeed = 5600;
            
            else if (ty < -13.55 && ty >= -14.28)
                limelightSpeed = 5900;
            
            else if (ty < -14.28 && ty >= 15)
                limelightSpeed = 6300;
            
            else
                limelightSpeed = 0;
            
        }
    }
}

        void Shooter::activateConveyor()
        {
            shooterConveyor.indexMotor.Set(0.9);
            if (trenchButtonPressed)
                shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);

            else if (initButtonPressed)
                shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);

            else if (wallButtonPressed)
                shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);

            else if (limelightButtonPressed)
                shooterConveyor.conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
        }
        void Shooter::modifyVelocity()
        {
            double modValue = 3.4133;
            double flyWheelDesiredSpeed = 0;

            if (trenchButtonPressed)
            {
                flyWheelDesiredSpeed = trenchSpeed * modValue;
            }
            else if (initButtonPressed)
            {
                flyWheelDesiredSpeed = initSpeed * modValue;
            }
            else if (wallButtonPressed)
            {
                flyWheelDesiredSpeed = wallSpeed * modValue;
            }
            else if (limelightButtonPressed)
            {
                flyWheelDesiredSpeed = limelightSpeed * modValue;
            }
            else
            {
                flyWheelDesiredSpeed = 0;
            }

            shooterMotorL.Set(TalonFXControlMode::Velocity, flyWheelDesiredSpeed);
        }

        void Shooter::setHoodPosition()
        {
            //Fill out when we have the actuator Docs
        }