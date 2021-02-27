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

#include <frc/Drive/RobotDriveBase.h>

//#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Drivetrain.h"

WPI_TalonFX IntakeMotor{0};

frc::Timer timer;
frc::Timer arrivedTimer;
frc::AnalogInput brAnalog{3};
frc::AnalogInput blAnalog{2};
frc::AnalogInput frAnalog{0};
frc::AnalogInput flAnalog{1};

class Robot : public frc::TimedRobot
{

public:
  int phase;

  void AutonomousInit() override
  {
    m_swerve.m_frontLeft.zeroDriveEncoder();
    m_swerve.m_frontRight.zeroDriveEncoder();
    m_swerve.m_backLeft.zeroDriveEncoder();
    m_swerve.m_backRight.zeroDriveEncoder();

    //resetTurnEncoder();

    m_swerve.ahrs->ZeroYaw();

    phase = 1;
    timer.Reset();
    timer.Start();

    arrivedTimer.Reset();
    arrivedTimer.Start();
    

    m_swerve.arrived = false;

    frc::Wait(0.05);
  }

  double pauseTime = 0.5;

  void AutonomousPeriodic() override
  {
    if (m_swerve.arrived == true)
    {
      if (arrivedTimer.Get() > pauseTime) {
        m_swerve.m_frontLeft.zeroDriveEncoder();
        m_swerve.m_frontRight.zeroDriveEncoder();
        m_swerve.m_backLeft.zeroDriveEncoder();
        m_swerve.m_backRight.zeroDriveEncoder();

        frc::Wait(0.02);

        phase++;
        std::cout << "phase increased" << std::endl;
        m_swerve.arrived = false;
      }
    }
    else
    {
     arrivedTimer.Reset();
    }

    std::cout << "arrived " << m_swerve.arrived << "  phase " << phase << "  drive encoder " << m_swerve.m_frontLeft.m_driveEncoder.GetPosition() << std::endl;

    barrelRaceAuto();
    //m_swerve.UpdateOdometry();
  }

  void TeleopInit() override
  {
    m_swerve.ahrs->ZeroYaw();
    /*Setting the encoders to mathc the position of the Gyro*/
    resetTurnEncoder();

  }

  int first_loop = 0;
  void TeleopPeriodic() override
  {
    if(first_loop == 0){
      resetTurnEncoder();
      first_loop = 1;
      frc::Wait(0.02);
      }
    //make a button to reset encoder values
    DriveWithJoystick(true); //true = field relative, false = not field relative
    frc::SmartDashboard::PutNumber("fr encoder", frAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("fl encoder", flAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("bl encoder", blAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("br encoder", brAnalog.GetVoltage());
  }

private:
  frc::Joystick m_controller{0};

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
  int rotationCounter = 0;
  bool resetEncoder = false;
  double gyroOld;

  double flturnStay = m_swerve.m_frontLeft.m_turningEncoder.GetPosition();
  double frturnStay = m_swerve.m_frontRight.m_turningEncoder.GetPosition();
  double blturnStay = m_swerve.m_backLeft.m_turningEncoder.GetPosition();
  double brturnStay = m_swerve.m_backRight.m_turningEncoder.GetPosition();

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
      resetTurnEncoder();
      resetEncoder = true;
    }
    else if (lBumper == false && resetEncoder == true)
    {
      resetEncoder = false;
    }

    /*if (leftX == 0 && leftY == 0 && rightX == 0)
    {
      m_swerve.m_frontLeft.m_turningEncoder.SetPosition(flturnStay);
      m_swerve.m_frontRight.m_turningEncoder.SetPosition(frturnStay);
      m_swerve.m_backLeft.m_turningEncoder.SetPosition(blturnStay);
      m_swerve.m_backRight.m_turningEncoder.SetPosition(brturnStay);
    }*/ //put something different in if statement

    auto xSpeed = /*strafe*/ m_xspeedLimiter.Calculate(leftX) * Drivetrain::kMaxSpeed;

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = /*-forward*/ -m_yspeedLimiter.Calculate(leftY) * Drivetrain::kMaxSpeed;
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left

    frc::SmartDashboard::PutNumber("doubleGyro", m_swerve.doubleGyro());

    if (rightX == 0)
    { //maybe works now
      const double kProt = 2.5; //6, 5 has a bit less flutter
      double currentRot = m_swerve.doubleGyro() + rotationCounter * 360;

      if ((rotSetpoint - currentRot) > 180)
      {
        rotationCounter++;
      }
      else if ((rotSetpoint - currentRot) < -180)
      {
        rotationCounter--;
      }

      double rotError = (rotSetpoint - currentRot);

      if (abs(rotError) < 1)
      {
        rotError = 0;
      }

      //frc::SmartDashboard::PutNumber("rotSetpoint", rotSetpoint);
      frc::SmartDashboard::PutNumber("doubleGyro", m_swerve.doubleGyro());
      //frc::SmartDashboard::PutNumber("rotError", rotError);
      //frc::SmartDashboard::PutNumber("currentRot", currentRot);

      if (rotError > 45)
      {
        rotError = 45;
      } 
      else if (rotError < -45)
      {
        rotError = -45;
      }

      auto rotStay = m_rotLimiter.Calculate((rotError / 180) * kProt) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rotStay, fieldRelative, false, 0, 0); //field relative = true
    }
    else
    {
      auto rot = m_rotLimiter.Calculate(rightX) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0);  //field relative = true

      rotSetpoint = m_swerve.doubleGyro() /* + rotationCounter * 360*/; //maybe this should just be m_swerve.doubleGyro();
    }

    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true
    gyroOld = m_swerve.doubleGyro();
  }

  void DriveAutonomous(double direction, double distance, double maxVelocity, bool fieldRelative)
  {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    /*double gyroDegrees = m_swerve.ahrs->GetYaw() * -1;
    float gyroRadians = gyroDegrees * (wpi::math::pi / 180);
    float forward = leftY * cos(gyroRadians) + leftX * sin(gyroRadians);
    float strafe = -leftY * sin(gyroRadians) + leftX * cos(gyroRadians);*/

    auto xSpeed = /*strafem_xspeedLimiter.Calculate(leftX) * Drivetrain::kMaxSpeed;*/ cos(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;

    //frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = /*-forward-m_yspeedLimiter.Calculate(leftY) * Drivetrain::kMaxSpeed*/ sin(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;

    //frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left

    const double kProt = 5;
    double currentRot = m_swerve.doubleGyro();

    if ((rotSetpoint - currentRot) > 180)
    {
      rotationCounter++;
    }
    else if ((rotSetpoint - currentRot) < -180)
    {
      rotationCounter--;
    }

    double rotError = (rotSetpoint - currentRot);

    if (abs(rotError) < .5)
    {
      rotError = 0;
    }

    //frc::SmartDashboard::PutNumber("rotSetpoint", rotSetpoint);
    //frc::SmartDashboard::PutNumber("doubleGyro", m_swerve.doubleGyro());
    //frc::SmartDashboard::PutNumber("rotError", rotError);
    //frc::SmartDashboard::PutNumber("currentRot", currentRot);

    auto rot = m_rotLimiter.Calculate((rotError / 180) * kProt) * Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, true, distance, maxVelocity); //field relative = true
  }

  void pauseAuto(double pauseTime)
  {
    m_swerve.m_frontLeft.pauseModule(pauseTime);
    m_swerve.m_frontRight.pauseModule(pauseTime);
    m_swerve.m_backLeft.pauseModule(pauseTime);
    m_swerve.m_backRight.pauseModule(pauseTime);
  }

  void slalomAuto()
  {
    if (phase == 1)
    { //1 -> 2
      DriveAutonomous(0, 60, 800, true);
    }
    else if (phase == 2)
    { //2 -> 3
      DriveAutonomous(90, 60, 800, true);
    }
    else if (phase == 3)
    { //3 -> 4
      DriveAutonomous(0, 180, 800, true);
    }
    else if (phase == 4)
    { //4 -> 5
      DriveAutonomous(-90, 60, 800, true);
    }
    else if (phase == 5)
    { //5 -> 6
      DriveAutonomous(0, 60, 800, true);
    }
    else if (phase == 6)
    { //6 -> 7
      DriveAutonomous(90, 60, 800, true);
    }
    else if (phase == 7)
    { //7 -> 8
      DriveAutonomous(180, 60, 800, true);
    }
    else if (phase == 8)
    { //8 -> 9
      DriveAutonomous(270, 60, 800, true);
    }
    else if (phase == 9)
    { //9 -> 10
      DriveAutonomous(180, 180, 800, true);
    }
    else if (phase == 10)
    { //10 -> 11
      DriveAutonomous(90, 60, 800, true);
    }
    else if (phase == 11)
    { //11 -> 12
      DriveAutonomous(180, 60, 800, true);
    }
  }

  void barrelRaceAuto()
  { //directions revised to have battery facing forward instead of to the right
    if (phase == 1)
    { //1 -> 2
      //pauseTime = 0.5;
      DriveAutonomous(90, 150, 3000, true); //double direction, double distance, double maxVelocity, bool fieldRelative
    }
    else if (phase == 2)
    { //2 -> 3
      //pauseAuto(1);
      DriveAutonomous(0, 60/*48*/, 3000, true);
    }
    else if (phase == 3)
    { //3 -> 4
      DriveAutonomous(-90, 60/*48*/, 3000, true);
    }
    else if (phase == 4)
    { //4 -> 5
      DriveAutonomous(-180, 48, 3000, true);
    }
    else if (phase == 5)
    { //5 -> 6
      DriveAutonomous(90, 150/*138*/, 3000, true);
    }
    else if (phase == 6)
    { //6 -> 7
      DriveAutonomous(180, 60/*48*/, 3000, true);
    }
    else if (phase == 7)
    { //7 -> 8
      DriveAutonomous(-90, 60/*48*/, 3000, true);
    }
    else if (phase == 8)
    { //8 -> 9
      DriveAutonomous(0, 120/*96*/, 3000, true);
    }
    else if (phase == 9)
    { //9 -> 10
      DriveAutonomous(90, 120/*108*/, 3000, true);
    }
    else if (phase == 10)
    { //10 -> 11
      DriveAutonomous(180, 60/*48*/, 3000, true);
    }
    else if (phase == 11)
    { //11 -> 12
      DriveAutonomous(-90, 312/*300*/, 3000, true);
      pauseTime = 5;
    }
    else if (phase == 12)
    { //12 -> 13
      DriveAutonomous(0, 0, 0, true);
    }
  }

  void bounceAuto()
  {
    if (phase == 1)
    {  //1 -> 2
      DriveAutonomous(0, 30, 800, true); //double direction, double distance, double maxVelocity, bool fieldRelative
    }
    else if (phase == 2)
    { //2 -> 3
      DriveAutonomous(90, 60, 800, true);
    }
    else if (phase == 3)
    { //3 -> 4
      DriveAutonomous(-90, 60, 800, true);
    }
    else if (phase == 4)
    { //4 -> 5
      DriveAutonomous(0, 48, 800, true);
    }
    else if (phase == 5)
    { //5 -> 6
      DriveAutonomous(-90, 48, 800, true);
    }
    else if (phase == 6)
    { //6 -> 7
      DriveAutonomous(0, 42, 800, true);
    }
    else if (phase == 7)
    { //7 -> 8
      DriveAutonomous(90, 108, 800, true);
    }
    else if (phase == 8)
    { //8 -> 9
      DriveAutonomous(-90, 108, 800, true);
    }
    else if (phase == 9)
    { //9 -> 10
      DriveAutonomous(0, 90, 800, true);
    }
    else if (phase == 10)
    { //10 -> 11
      DriveAutonomous(90, 108, 800, true);
    }
    else if (phase == 11)
    { //11 -> 12
      DriveAutonomous(-90, 60, 800, true);
    }
    else if (phase == 11)
    { //12 -> 13
      DriveAutonomous(0, 72, 800, true);
    }
  }

  double fl_relativeEncoder;
  double fr_relativeEncoder;
  double bl_relativeEncoder;
  double br_relativeEncoder; 
  void resetTurnEncoder()
{

  frc::SmartDashboard::PutNumber("Front Left Encoder Value", fl_relativeEncoder);
    /*The following gets the gyro angle, compares it to where it is on the circle and then translates that to the
    encoder
    TODO: Wrong Math, need to implement the analog encoder in some way*/

    /*PROBLEM:Currently snapping 45 degrees instead of zeroing*/
    if (m_swerve.doubleGyro() <= 180)
    {
      fl_relativeEncoder = (-9/180) * m_swerve.doubleGyro();
    }
    else if (m_swerve.doubleGyro() > 180)
    {
      fl_relativeEncoder = ((-9/180) *  m_swerve.doubleGyro()) + 18; //changed 360 to 18
    }

    m_swerve.m_frontLeft.m_turningEncoder.SetPosition(fl_relativeEncoder);

    if (m_swerve.doubleGyro() <= 180)
    {
      fr_relativeEncoder = (-9/180) * m_swerve.doubleGyro();
    }
    else if (m_swerve.doubleGyro() > 180)
    {
      fr_relativeEncoder = ((-9/180) *  m_swerve.doubleGyro()) + 18; 
    }

    m_swerve.m_frontRight.m_turningEncoder.SetPosition(fr_relativeEncoder);

    if (m_swerve.doubleGyro() <= 180)
    {
      bl_relativeEncoder = (-9/180) * m_swerve.doubleGyro();
    }
    else if (m_swerve.doubleGyro() > 180)
    {
      bl_relativeEncoder = ((-9/180) *  m_swerve.doubleGyro()) + 18;
    }

    m_swerve.m_backLeft.m_turningEncoder.SetPosition(bl_relativeEncoder);

    if (m_swerve.doubleGyro() <= 180)
    {
      br_relativeEncoder = (-9/180) * m_swerve.doubleGyro();
    }
    else if (m_swerve.doubleGyro() > 180)
    {
      br_relativeEncoder = ((-9/180) *  m_swerve.doubleGyro()) + 18;
    }

    m_swerve.m_backRight.m_turningEncoder.SetPosition(br_relativeEncoder);

    std::cout << "encoder reset" << std::endl;
    frc::Wait(0.02);
}

};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}



#endif