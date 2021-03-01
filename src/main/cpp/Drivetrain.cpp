#include "Drivetrain.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <stdio.h>

void Drivetrain::zeroDriveEncoders()
{
  m_frontLeft.zeroDriveEncoder();
  m_frontRight.zeroDriveEncoder();
  m_backLeft.zeroDriveEncoder();
  m_backRight.zeroDriveEncoder();
}

void Drivetrain::displayAnalogEncoders()
{
  frc::SmartDashboard::PutNumber("FR Encoder", frontRightAnalogEncoder.GetVoltage());
  frc::SmartDashboard::PutNumber("FL Encoder", frontLeftAnalogEncoder.GetVoltage());
  frc::SmartDashboard::PutNumber("BL Encoder", backLeftAnalogEncoder.GetVoltage());
  frc::SmartDashboard::PutNumber("BR Encoder", backRightAnalogEncoder.GetVoltage());
}

void Drivetrain::correctRotation()
{
  rotSetpoint = doubleGyro();
  currentRot = doubleGyro() + rotationCounter * 360;

  if ((rotSetpoint - currentRot) > 180)
  {
    rotationCounter++;
  }
  else if ((rotSetpoint - currentRot) < -180)
  {
    rotationCounter--;
  }
}

void Drivetrain::findRotError()
{
  rotError = (rotSetpoint - currentRot);

  if (abs(rotError) < 1)
  {
    rotError = 0;
  }

  // Keeps the Rotation Error in-bounds
  if (rotError > 45)
  {
    rotError = 45;
  }
  else if (rotError < -45)
  {
    rotError = -45;
  }
}

void Drivetrain::printRotValues()
{
  //frc::SmartDashboard::PutNumber("rotSetpoint", rotSetpoint);
  frc::SmartDashboard::PutNumber("doubleGyro", doubleGyro());
  //frc::SmartDashboard::PutNumber("rotError", rotError);
  //frc::SmartDashboard::PutNumber("currentRot", currentRot);
}

void Drivetrain::debugModuleAngles()
{
  frc::SmartDashboard::PutNumber("FLCurrentAngle", m_frontLeft.GetAngle());
  frc::SmartDashboard::PutNumber("FRCurrentAngle", m_frontRight.GetAngle());
  frc::SmartDashboard::PutNumber("BLCurrentAngle", m_backLeft.GetAngle());
  frc::SmartDashboard::PutNumber("BRCurrentAngle", m_backRight.GetAngle());
}

void Drivetrain::defineStates(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative)
{
  auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetAngle()) : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);
  auto [frontLeftState, frontRightState, backLeftState, backRightState] = states;
}

void Drivetrain::retainAngle()
{
  frontLeftState.angle = previousFrontLeftState.angle;
  frontRightState.angle = previousFrontRightState.angle;
  backLeftState.angle = previousBackLeftState.angle;
  backRightState.angle = previousBackRightState.angle;
}

void Drivetrain::keepEncoderAngle() // Outdated, but it might work with some modifications
{
  // This block will need a separate function, but that's a future me problem
  double flturnStay = m_frontLeft.m_turningEncoder.GetPosition();
  double frturnStay = m_frontRight.m_turningEncoder.GetPosition();
  double blturnStay = m_backLeft.m_turningEncoder.GetPosition();
  double brturnStay = m_backRight.m_turningEncoder.GetPosition();

  m_frontLeft.m_turningEncoder.SetPosition(flturnStay);
  m_frontRight.m_turningEncoder.SetPosition(frturnStay);
  m_backLeft.m_turningEncoder.SetPosition(blturnStay);
  m_backRight.m_turningEncoder.SetPosition(brturnStay);
}

void Drivetrain::resetTurnEncoder()
{

  frc::SmartDashboard::PutNumber("Front Left Encoder Value", frontLeftRelativeEncoder);

  /*The following gets the gyro angle, compares it to where it is on the circle and then translates that to the
    encoder
    
    TODO: Wrong Math, need to implement the analog encoder in some way*/

  /*PROBLEM:Currently snapping 45 degrees instead of zeroing*/

  // This could use a better name, but I can't think of one atm
  double relativeEncoderConversion = (-9 / 180) * doubleGyro();

  if (doubleGyro() <= 180)
  {
    frontLeftRelativeEncoder = relativeEncoderConversion;
    backRightRelativeEncoder = relativeEncoderConversion;
    backLeftRelativeEncoder = relativeEncoderConversion;
    frontRightRelativeEncoder = relativeEncoderConversion;
  }
  else if (doubleGyro() > 180)
  {
    frontLeftRelativeEncoder = relativeEncoderConversion + 18; //changed 360 to 18
    backRightRelativeEncoder = relativeEncoderConversion + 18;
    backLeftRelativeEncoder = relativeEncoderConversion + 18;
    frontRightRelativeEncoder = relativeEncoderConversion + 18;
  }

  m_frontLeft.m_turningEncoder.SetPosition(frontLeftRelativeEncoder);
  m_frontRight.m_turningEncoder.SetPosition(frontRightRelativeEncoder);
  m_backLeft.m_turningEncoder.SetPosition(backLeftRelativeEncoder);
  m_backRight.m_turningEncoder.SetPosition(backRightRelativeEncoder);

  std::printf("Encoders Reset\n");
  frc::Wait(0.02);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, bool autonomous = false, double distance = 0, double maxVelocity = 0)
{
  defineStates(xSpeed, ySpeed, rot, fieldRelative);

  frontLeftState.speed *= driveSpeedMod;
  frontRightState.speed *= driveSpeedMod;
  backLeftState.speed *= driveSpeedMod;
  backRightState.speed *= driveSpeedMod;

  units::dimensionless::scalar_t totalModules = 4;
  units::dimensionless::scalar_t average = (frontLeftState.speed.to<double>() + frontRightState.speed.to<double>() +
                                            backLeftState.speed.to<double>() + backRightState.speed.to<double>()) /
                                           totalModules;
  units::dimensionless::scalar_t linearFix{0}; //the lower it is, the more squared the drive is, range from 0 to 1

  /*The following is used to change the sensitivity of the joystick according to the linear fix variable
  When Linearfix = 1, joystick to power is y=x. When Linearfix = 0, the joystick to power is y=x^2*/

  frontLeftState.speed = (linearFix * frontLeftState.speed) + ((1 - linearFix) * ((average)*frontLeftState.speed));
  frontRightState.speed = (linearFix * frontRightState.speed) + ((1 - linearFix) * ((average)*frontRightState.speed));
  backLeftState.speed = (linearFix * backLeftState.speed) + ((1 - linearFix) * ((average)*backLeftState.speed));
  backRightState.speed = (linearFix * backRightState.speed) + ((1 - linearFix) * ((average)*backRightState.speed));

  // If not moving, keep the last angle
  if (xSpeed.to<double>() == 0 && ySpeed.to<double>() == 0 && rot.to<double>() == 0)
  {
    retainAngle();
  }

  previousFrontLeftState.angle = frontLeftState.angle;
  previousFrontRightState.angle = frontRightState.angle;
  previousBackLeftState.angle = backLeftState.angle;
  previousBackRightState.angle = backRightState.angle;

  flb = m_frontLeft.SetDesiredState(frontLeftState, autonomous, distance, maxVelocity);
  frb = m_frontRight.SetDesiredState(frontRightState, autonomous, distance, maxVelocity);
  blb = m_backLeft.SetDesiredState(backLeftState, autonomous, distance, maxVelocity);
  brb = m_backRight.SetDesiredState(backRightState, autonomous, distance, maxVelocity);

  if (flb + frb + blb + brb > 0)
  {
    arrived = true;
  }
  else
  {
    arrived = false;
  }

  //debugEncoderAngle();
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(GetAngle(), m_frontLeft.GetState(), m_frontRight.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());
}