/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "SwerveModule.h"
#include <frc/Talon.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/Phoenix.h>
#include "Autons.h"

#include <frc/Drive/RobotDriveBase.h>

#include <frc/Joystick.h>

#include "Drivetrain.h"

WPI_TalonFX IntakeMotor{0};

frc::AnalogInput brAnalog{3};
frc::AnalogInput blAnalog{2};
frc::AnalogInput frAnalog{0};
frc::AnalogInput flAnalog{1};

class Robot : public frc::TimedRobot
{

public:
  void AutonomousInit() override
  {
    Autonomous.Initialize();
  }

  void AutonomousPeriodic() override
  {
    Autonomous.pauseLogic();

    std::cout << "arrived " << m_swerve.arrived << "  phase " << Autonomous.phase 
              << "  drive encoder " << m_swerve.m_frontLeft.m_driveEncoder.GetPosition() << std::endl;

    Autonomous.barrelRaceAuto();
    //Autonomous.autonDrive.UpdateOdometry();
  }

  void TeleopInit() override
  {
    m_swerve.swerveGyro->ZeroYaw();
    /*Setting the encoders to mathc the position of the Gyro*/
    m_swerve.resetTurnEncoder();
  }

  int first_loop = 0;
  void TeleopPeriodic() override
  {
    if (first_loop == 0)
    {
      m_swerve.resetTurnEncoder();
      first_loop = 1;
      frc::Wait(0.02);
    }
    //make a button to reset encoder values
    DriveWithJoystick(true); //true = field relative, false = not field relative
    m_swerve.displayAnalogEncoders();
  }

private:
  frc::Joystick m_controller{0};
  Autons Autonomous;

  //joystick defines
  double leftX{m_controller.GetRawAxis(0)};
  double leftY{m_controller.GetRawAxis(1)};
  double rightX{m_controller.GetRawAxis(4)}; //4 on gamepad

  double lTrigger{m_controller.GetRawAxis(2)};
  bool lBumper{m_controller.GetRawButton(5)};
  bool rBumper{m_controller.GetRawButton(6)};

  double deadzone = 0.05;

  Drivetrain m_swerve;
  units::velocity::meters_per_second_t prev_x_speed;
  units::velocity::meters_per_second_t prev_y_speed;
  units::angular_velocity::radians_per_second_t prev_rot_speed;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  double rotSetpoint = m_swerve.doubleGyro();
  bool resetEncoder = false;

  void DriveWithJoystick(bool fieldRelative)
  {
    lBumper = m_controller.GetRawButton(5);
    lTrigger = m_controller.GetRawAxis(2);

    leftX = m_controller.GetRawAxis(0);
    leftY = m_controller.GetRawAxis(1);
    rightX = -m_controller.GetRawAxis(4);

    if (leftY < deadzone && leftY > (deadzone * -1) && leftX < deadzone && leftX > (deadzone * -1))
    {
      leftY = 0;
      leftX = 0;
    }

    if (rightX < deadzone && rightX > (deadzone * -1))
    {
      rightX = 0;
    }

    if (lTrigger > 0)
    {
      IntakeMotor.Set(ControlMode::PercentOutput, -lTrigger);
    }
    else
    {
      IntakeMotor.Set(0);
    }

    if (lBumper == true && resetEncoder == false)
    {
      m_swerve.resetTurnEncoder();
      resetEncoder = true;
    }
    else if (lBumper == false && resetEncoder == true)
    {
      resetEncoder = false;
    }

    /* This still needs something else, but I create a template for when we correct it
    if (leftX == 0 && leftY == 0 && rightX == 0)
    {
      m_swerve.keepEncoderAngle();
    } 
    */

    auto xSpeed = /*strafe*/ m_xspeedLimiter.Calculate(leftX) * Drivetrain::kMaxSpeed;

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());

    // Get the y speed or forward/back speed. We are inverting this because
    // our controllers  return positive values when you pull to the back by default
    auto ySpeed = /*-forward*/ -m_yspeedLimiter.Calculate(leftY) * Drivetrain::kMaxSpeed;
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left
    frc::SmartDashboard::PutNumber("doubleGyro", m_swerve.doubleGyro());

    if (rightX == 0)
    {
      m_swerve.correctRotation();
      m_swerve.findRotError();

      m_swerve.printRotValues();

      auto rotStay = m_rotLimiter.Calculate((m_swerve.rotError / 180) * m_swerve.ROTATION_P) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rotStay, fieldRelative, false, 0, 0); //field relative = true
    }
    else
    {
      auto rot = m_rotLimiter.Calculate(rightX) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true

      rotSetpoint = m_swerve.doubleGyro() /* + rotationCounter * 360*/;
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}

#endif