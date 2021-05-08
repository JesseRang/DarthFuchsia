/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>
#include <iostream>
#include <AHRS.h>
#include <frc/livewindow/LiveWindow.h>
#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  NetworkTable *table;
  frc::LiveWindow *lw = NULL;
  AHRS *ahrs = new AHRS(frc::SPI::Port::kMXP);
  double start_gyro = ahrs->GetAngle();
  Drivetrain() {  ahrs->ZeroYaw(); }

  frc::SwerveModuleState pfl;
  frc::SwerveModuleState pfr;
  frc::SwerveModuleState pbl;
  frc::SwerveModuleState pbr;

  SwerveModule m_frontRight{1, 2, start_gyro};
  SwerveModule m_frontLeft{3, 4, start_gyro};
  SwerveModule m_backLeft{5, 6, start_gyro};
  SwerveModule m_backRight{7, 8, start_gyro};

  int flb;
  int frb;
  int blb;
  int brb;

  bool arrived = false;

  /**
   * Get the robot angle as a Rotation2d.
   */
  frc::Rotation2d GetAngle() const {
    // Negating the angle because WPILib Gyros are CW positive.
    return frc::Rotation2d(units::degree_t(-ahrs->GetYaw()));
  }

  double doubleGyro() const {
    // Negating the angle because WPILib Gyros are CW positive.
    return (-ahrs->GetYaw());
  }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, bool autonomous, double distance, double maxVelocity);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.65_mps;  // 3.65 meters per second

  static constexpr units::radians_per_second_t kMinAngularSpeed{
      wpi::math::pi * 1.5};  // 1/2 * 1.5 rotation per second

  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi * 2};  // 1/2 * 4 rotation per second

 private:
  frc::Translation2d m_frontLeftLocation{-0.305_m, +0.305_m};
  frc::Translation2d m_frontRightLocation{+0.305_m, +0.305_m};
  frc::Translation2d m_backLeftLocation{-0.305_m, -0.305_m};
  frc::Translation2d m_backRightLocation{+0.305_m, -0.305_m};


 
  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, GetAngle()};
};