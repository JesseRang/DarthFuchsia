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

#include <frc/Joystick.h>

#include "Drivetrain.h"
#include "Intake.h"
#include "Shooter.h"

WPI_TalonFX IntakeMotor{0};

frc::Timer timer;
frc::Timer arrivedTimer;
frc::AnalogInput brAnalog{3};
frc::AnalogInput blAnalog{2};
frc::AnalogInput frAnalog{0};
frc::AnalogInput flAnalog{1};

frc::DoubleSolenoid intakeSolenoid{2, 3};

class Robot : public frc::TimedRobot
{
public:
  Shooter driveShooter;
  Intake driveIntake;
  int phase;

  void AutonomousInit() override
  {
    m_swerve.m_frontLeft.zeroDriveEncoder();
    m_swerve.m_frontRight.zeroDriveEncoder();
    m_swerve.m_backLeft.zeroDriveEncoder();
    m_swerve.m_backRight.zeroDriveEncoder();

    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);

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
      if (arrivedTimer.Get() > pauseTime)
      {
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
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    driveShooter.Initiate();
    driveIntake.Initiate();
    m_swerve.ahrs->ZeroYaw();
    m_swerve.m_frontLeft.zeroTurnEncoder();
    m_swerve.m_frontRight.zeroTurnEncoder();
    m_swerve.m_backLeft.zeroTurnEncoder();
    m_swerve.m_backRight.zeroTurnEncoder();
    /*Setting the encoders to mathc the position of the Gyro*/
    //resetTurnEncoder();
  }

  int first_loop = 0;
  bool isTogglingIntake;
  bool solenoidUp = false;

  bool firstTogglePress = true;

  void TeleopPeriodic() override
  {
    driveShooter.updateButtons();
    driveIntake.updateButtons();
    isTogglingIntake = driveIntake.driverController.GetRawButton(1);
    //make a button to reset encoder values
    //DriveWithJoystick(false); //true = field relative, false = not field relative
    frc::SmartDashboard::PutNumber("fr encoder", frAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("fl encoder", flAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("bl encoder", blAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("br encoder", brAnalog.GetVoltage());

    driveIntake.Run();

    if (isTogglingIntake)
    {
      if (solenoidUp && firstTogglePress)
      {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        solenoidUp = false;
  
      }
      else if (!solenoidUp && firstTogglePress)
      {

        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        solenoidUp = true;
      }
      firstTogglePress = false;
    }
    else
    {
      firstTogglePress = true;
    }
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

    /*if (lBumper == true && resetEncoder == false)
    {
      resetTurnEncoder();
      resetEncoder = true;
    }
    else if (lBumper == false && resetEncoder == true)
    {
      resetEncoder = false;
    }*/

    /*if (leftX == 0 && leftY == 0 && rightX == 0)
    {
      m_swerve.m_frontLeft.m_turningEncoder.SetPosition(flturnStay);
      m_swerve.m_frontRight.m_turningEncoder.SetPosition(frturnStay);
      m_swerve.m_backLeft.m_turningEncoder.SetPosition(blturnStay);
      m_swerve.m_backRight.m_turningEncoder.SetPosition(brturnStay);
    }*/
    //put something different in if statement

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

    /*if (rightX == 0)
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

      auto rotStay = m_rotLimiter.Calculate(/*(rotError / 180) * kProt rightX) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rotStay, fieldRelative, false, 0, 0); //field relative = true
    }
    else
    {*/
    auto rot = m_rotLimiter.Calculate(rightX) * Drivetrain::kMaxAngularSpeed;
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true

    rotSetpoint = m_swerve.doubleGyro() /* + rotationCounter * 360*/; //maybe this should just be m_swerve.doubleGyro();
    //}

    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true
    gyroOld = m_swerve.doubleGyro();

    frc::SmartDashboard::PutNumber("fl motor encoder", m_swerve.m_frontLeft.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("fr motor encoder", m_swerve.m_frontRight.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("bl motor encoder", m_swerve.m_backLeft.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("br motor encoder", m_swerve.m_backRight.m_turningEncoder.GetPosition());
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
      DriveAutonomous(0, 60 /*48*/, 3000, true);
    }
    else if (phase == 3)
    { //3 -> 4
      DriveAutonomous(-90, 60 /*48*/, 3000, true);
    }
    else if (phase == 4)
    { //4 -> 5
      DriveAutonomous(-180, 48, 3000, true);
    }
    else if (phase == 5)
    { //5 -> 6
      DriveAutonomous(90, 150 /*138*/, 3000, true);
    }
    else if (phase == 6)
    { //6 -> 7
      DriveAutonomous(180, 60 /*48*/, 3000, true);
    }
    else if (phase == 7)
    { //7 -> 8
      DriveAutonomous(-90, 60 /*48*/, 3000, true);
    }
    else if (phase == 8)
    { //8 -> 9
      DriveAutonomous(0, 120 /*96*/, 3000, true);
    }
    else if (phase == 9)
    { //9 -> 10
      DriveAutonomous(90, 120 /*108*/, 3000, true);
    }
    else if (phase == 10)
    { //10 -> 11
      DriveAutonomous(180, 60 /*48*/, 3000, true);
    }
    else if (phase == 11)
    { //11 -> 12
      DriveAutonomous(-90, 312 /*300*/, 3000, true);
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
    {                                    //1 -> 2
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
    /*STEP 1*/
    double gyroAngle = m_swerve.doubleGyro() + 180; //gets the heading from the Navx Board, 0 - 360
    /*STEP 2*/
    double resetPosition = gyroAngle / 180 * 9; //convert the angle to encoder clicks
    /*STEP 3*/
    /*m_swerve.m_frontLeft.m_turningEncoder.SetPosition(resetPosition); //set all the new encoder positions
    m_swerve.m_frontRight.m_turningEncoder.SetPosition(resetPosition);
    m_swerve.m_backLeft.m_turningEncoder.SetPosition(resetPosition);
    m_swerve.m_backRight.m_turningEncoder.SetPosition(resetPosition);*/
    /*STEP1 get distance to home which is 2.45*/
    double distToHome = flAnalog.GetVoltage() - 2.45;
    if (flAnalog.GetVoltage() <= 2.45)
    {
      fl_relativeEncoder = (-9 / 2.45) * flAnalog.GetVoltage();
    }
    else if (flAnalog.GetVoltage() > 2.45)
    {
      fl_relativeEncoder = ((-9 / 2.45) * flAnalog.GetVoltage()) + 18; //changed 360 to 18
    }

    m_swerve.m_frontLeft.m_turningEncoder.SetPosition(fl_relativeEncoder);

    if (frAnalog.GetVoltage() <= 2.45)
    {
      fr_relativeEncoder = (-9 / 2.45) * frAnalog.GetVoltage();
    }
    else if (frAnalog.GetVoltage() > 2.45)
    {
      fr_relativeEncoder = ((-9 / 2.45) * frAnalog.GetVoltage()) + 18;
    }

    m_swerve.m_frontRight.m_turningEncoder.SetPosition(fr_relativeEncoder);

    if (blAnalog.GetVoltage() <= 2.45)
    {
      bl_relativeEncoder = (-9 / 2.45) * blAnalog.GetVoltage();
    }
    else if (blAnalog.GetVoltage() > 2.45)
    {
      bl_relativeEncoder = ((-9 / 2.45) * blAnalog.GetVoltage()) + 18;
    }

    m_swerve.m_backLeft.m_turningEncoder.SetPosition(bl_relativeEncoder);

    if (brAnalog.GetVoltage() <= 2.45)
    {
      br_relativeEncoder = (-9 / 2.45) * brAnalog.GetVoltage();
    }
    else if (brAnalog.GetVoltage() > 2.45)
    {
      br_relativeEncoder = ((-9 / 2.45) * brAnalog.GetVoltage()) + 18;
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