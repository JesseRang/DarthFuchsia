#include "Drivetrain.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <stdio.h>

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, bool autonomous = 0, double distance = 0, double maxVelocity = 0)
{
  auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetAngle()) : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  fl.speed *= 1.25;
  fr.speed *= 1.25;
  bl.speed *= 1.25;
  br.speed *= 1.25;

  units::dimensionless::scalar_t totalModules = 4;
  units::dimensionless::scalar_t average = (fl.speed.to<double>() + fr.speed.to<double>() + bl.speed.to<double>() + br.speed.to<double>())/totalModules;
  units::dimensionless::scalar_t linearFix{0}; //the lower it is, the more squared the drive is, range from 0 to 1

  /*The following is used to change the sensitivity of the joystick according to the linear fix variable
  When Linearfix = 1, joystick to power is y=x. When Linearfix = 0, the joystick to power is y=x^2*/
  fl.speed = (linearFix * fl.speed) + ((1 - linearFix) * ((average) * fl.speed));
  fr.speed = (linearFix * fr.speed) + ((1 - linearFix) * ((average) * fr.speed));
  bl.speed = (linearFix * bl.speed) + ((1 - linearFix) * ((average) * bl.speed));
  br.speed = (linearFix * br.speed) + ((1 - linearFix) * ((average) * br.speed));
  
  if (xSpeed.to<double>() == 0 && ySpeed.to<double>() == 0 && rot.to<double>() == 0) { //added rot to prevent wheels not changing when trying to rotate
    fl.angle = pfl.angle;
    fr.angle = pfr.angle;
    bl.angle = pbl.angle;
    br.angle = pbr.angle;
  }

  pfl.angle = fl.angle;
  pfr.angle = fr.angle;
  pbl.angle = bl.angle;
  pbr.angle = br.angle;

  flb = m_frontLeft.SetDesiredState(fl, autonomous, distance, maxVelocity);
  frb = m_frontRight.SetDesiredState(fr, autonomous, distance, maxVelocity); 
  blb = m_backLeft.SetDesiredState(bl, autonomous, distance, maxVelocity); 
  brb = m_backRight.SetDesiredState(br, autonomous, distance, maxVelocity); 

  if (flb + frb + blb + brb > 0) {
    arrived = true;
  } else {
    arrived = false;
  } //for auton //used to be commented out

  //frc::SmartDashboard::PutNumber("FLCurrentAngle", m_frontLeft.GetAngle());
  //frc::SmartDashboard::PutNumber("FRCurrentAngle", m_frontRight.GetAngle());
  //frc::SmartDashboard::PutNumber("BLCurrentAngle", m_backLeft.GetAngle());
  //frc::SmartDashboard::PutNumber("BRCurrentAngle", m_backRight.GetAngle());
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(GetAngle(), m_frontLeft.GetState(), m_frontRight.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());
}