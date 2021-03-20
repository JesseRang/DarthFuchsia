/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/math>
#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/AnalogInput.h>
#define brushless rev::CANSparkMax::MotorType::kBrushless




class SwerveModule
{
public:
  int speedFlip = 1;
  bool flipping;
  double m_error;

  SwerveModule(int driveMotorChannel, int turningMotorChannel, double gyroStartAng);
  frc::SwerveModuleState GetState();

  double GetVelocity()
  {
    return m_driveEncoder.GetVelocity();
  }
  double GetAngle()
  {
    return m_turningEncoder.GetPosition();
  }
double PrintSetVelocity(const frc::SwerveModuleState &state)
  {
    return state.speed.to<double>() * 500;
  }

  void SetPID();
  bool SetDesiredState(const frc::SwerveModuleState &state, bool autonomous, double distance, double maxVelocity);

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  frc::AnalogInput fl_analog{1};
  frc::AnalogInput fr_analog{0};
  frc::AnalogInput bl_analog{2};
  frc::AnalogInput br_analog{3};

  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();
  rev::CANEncoder m_turningEncoder = m_turningMotor.GetEncoder();


  rev::CANPIDController m_drivePIDController = m_driveMotor.GetPIDController();
  rev::CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();

  void zeroDriveEncoder();
  void zeroTurnEncoder();
  bool pauseModule(double time);

private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kEncoderResolution = 42;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1_rad_per_s; // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s; // radians per second^2
  double lastCommand = 0;
};