#include "Drivetrain.h"
#include <frc/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Autons.h"

void Autons::Initialize()
{
    autonDrive.zeroDriveEncoders();

    //autonDrive.resetTurnEncoder();
    
    autonDrive.swerveGyro->ZeroYaw();

    phase = 1;
    timer.Reset();
    timer.Start();

    pauseTimer.Reset();
    pauseTimer.Start();

    autonDrive.arrived = false;

    frc::Wait(0.05);
}

void Autons::pauseLogic()
{
    if (autonDrive.arrived == true)
    {
      if (pauseTimer.Get() > pauseTime)
      {
        autonDrive.zeroDriveEncoders();
        frc::Wait(0.02);

        phase++;
        std::printf("Phase increased\n");
        autonDrive.arrived = false;
      }
    }
    else
    {
      pauseTimer.Reset();
    }
}

void Autons::debugRotationMath()
{
    frc::SmartDashboard::PutNumber("rotSetpoint", rotSetpoint);
    frc::SmartDashboard::PutNumber("doubleGyro", autonDrive.doubleGyro());
    frc::SmartDashboard::PutNumber("rotError", rotError);
    frc::SmartDashboard::PutNumber("currentRot", currentRot);
}

void Autons::pauseAuto(double pauseTime)
{
    autonDrive.m_frontLeft.pauseModule(pauseTime);
    autonDrive.m_frontRight.pauseModule(pauseTime);
    autonDrive.m_backLeft.pauseModule(pauseTime);
    autonDrive.m_backRight.pauseModule(pauseTime);
}

void Autons::DriveAutonomous(double direction, double distance, double maxVelocity, bool fieldRelative = true)
{
    // Get the x and y speeds.
    auto xSpeed = /* Drivetrain::kMaxSpeed;*/ cos(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;
    auto ySpeed = /*Drivetrain::kMaxSpeed*/ sin(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());

    // Get the rate of angular rotation.

    currentRot = autonDrive.doubleGyro() + rotationCounter * 360;

    if ((rotSetpoint - currentRot) > 180)
    {
        rotationCounter++;
    }
    else if ((rotSetpoint - currentRot) < -180)
    {
        rotationCounter--;
    }

    rotError = (rotSetpoint - currentRot);

    if (abs(rotError) < 0.5)
    {
        rotError = 0;
    }

    //debugRotationMath();

    auto finalRot = m_rotLimiter.Calculate((rotError / 180) * ROTATION_P) * Drivetrain::kMaxAngularSpeed;

    autonDrive.Drive(xSpeed, ySpeed, finalRot, fieldRelative, true, distance, maxVelocity); //field relative = true
}

void Autons::barrelRaceAuto()
{
    if (phase == 1)
    { //1 -> 2
        //pauseTime = 0.5;
        DriveAutonomous(90, 150, barrelRaceMaxVeloctiy); //double direction, double distance, double maxVelocity, bool fieldRelative
    }
    else if (phase == 2)
    { //2 -> 3
        //pauseAuto(1);
        DriveAutonomous(0, 60 /*48*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 3)
    { //3 -> 4
        DriveAutonomous(-90, 60 /*48*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 4)
    { //4 -> 5
        DriveAutonomous(-180, 48, barrelRaceMaxVeloctiy);
    }
    else if (phase == 5)
    { //5 -> 6
        DriveAutonomous(90, 150 /*138*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 6)
    { //6 -> 7
        DriveAutonomous(180, 60 /*48*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 7)
    { //7 -> 8
        DriveAutonomous(-90, 60 /*48*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 8)
    { //8 -> 9
        DriveAutonomous(0, 120 /*96*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 9)
    { //9 -> 10
        DriveAutonomous(90, 120 /*108*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 10)
    { //10 -> 11
        DriveAutonomous(180, 60 /*48*/, barrelRaceMaxVeloctiy);
    }
    else if (phase == 11)
    { //11 -> 12
        DriveAutonomous(-90, 312 /*300*/, barrelRaceMaxVeloctiy);
        pauseTime = 5;
    }
    else if (phase == 12)
    { //12 -> 13
        DriveAutonomous(0, 0, 0);
    }
}

void Autons::slalomAuto()
{
    if (phase == 1)
    { //1 -> 2
        DriveAutonomous(0, 60, slalomMaxVelocity);
    }
    else if (phase == 2)
    { //2 -> 3
        DriveAutonomous(90, 60, slalomMaxVelocity);
    }
    else if (phase == 3)
    { //3 -> 4
        DriveAutonomous(0, 180, slalomMaxVelocity);
    }
    else if (phase == 4)
    { //4 -> 5
        DriveAutonomous(-90, 60, slalomMaxVelocity);
    }
    else if (phase == 5)
    { //5 -> 6
        DriveAutonomous(0, 60, slalomMaxVelocity);
    }
    else if (phase == 6)
    { //6 -> 7
        DriveAutonomous(90, 60, slalomMaxVelocity);
    }
    else if (phase == 7)
    { //7 -> 8
        DriveAutonomous(180, 60, slalomMaxVelocity);
    }
    else if (phase == 8)
    { //8 -> 9
        DriveAutonomous(270, 60, slalomMaxVelocity);
    }
    else if (phase == 9)
    { //9 -> 10
        DriveAutonomous(180, 180, slalomMaxVelocity);
    }
    else if (phase == 10)
    { //10 -> 11
        DriveAutonomous(90, 60, slalomMaxVelocity);
    }
    else if (phase == 11)
    { //11 -> 12
        DriveAutonomous(180, 60, slalomMaxVelocity);
    }
}

void Autons::bounceAuto()
{
    if (phase == 1)
    {                                              //1 -> 2
        DriveAutonomous(0, 30, bounceMaxVelocity); //double direction, double distance, double maxVelocity, bool fieldRelative
    }
    else if (phase == 2)
    { //2 -> 3
        DriveAutonomous(90, 60, bounceMaxVelocity);
    }
    else if (phase == 3)
    { //3 -> 4
        DriveAutonomous(-90, 60, bounceMaxVelocity);
    }
    else if (phase == 4)
    { //4 -> 5
        DriveAutonomous(0, 48, bounceMaxVelocity);
    }
    else if (phase == 5)
    { //5 -> 6
        DriveAutonomous(-90, 48, bounceMaxVelocity);
    }
    else if (phase == 6)
    { //6 -> 7
        DriveAutonomous(0, 42, bounceMaxVelocity);
    }
    else if (phase == 7)
    { //7 -> 8
        DriveAutonomous(90, 108, bounceMaxVelocity);
    }
    else if (phase == 8)
    { //8 -> 9
        DriveAutonomous(-90, 108, bounceMaxVelocity);
    }
    else if (phase == 9)
    { //9 -> 10
        DriveAutonomous(0, 90, bounceMaxVelocity);
    }
    else if (phase == 10)
    { //10 -> 11
        DriveAutonomous(90, 108, bounceMaxVelocity);
    }
    else if (phase == 11)
    { //11 -> 12
        DriveAutonomous(-90, 60, bounceMaxVelocity);
    }
    else if (phase == 11)
    { //12 -> 13
        DriveAutonomous(0, 72, bounceMaxVelocity);
    }
}