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
#include "LimeLight.h"

WPI_TalonFX IntakeMotor{0};

frc::DigitalInput breakBeamNewBall{6};       //right after the V
frc::DigitalInput breakBeamIndex{7};         //near the indexer wheel below the conveyor
frc::DigitalInput breakBeamConveyorStart{8}; //at the start of the conveyor
frc::DigitalInput breakBeamFull{9};          //at the top of the conveyor

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
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    driveShooter.Initiate();
    driveIntake.Initiate();
    m_swerve.ahrs->ZeroYaw();
    /*m_swerve.m_frontLeft.zeroTurnEncoder(4);
    m_swerve.m_frontRight.zeroTurnEncoder(2);
    m_swerve.m_backLeft.zeroTurnEncoder(6);
    m_swerve.m_backRight.zeroTurnEncoder(8);*/

    /*Setting the encoders to mathc the position of the Gyro*/
    resetTurnEncoder();
  }

  int first_loop = 0;
  bool isTogglingIntake;
  bool solenoidUp = false;

  bool firstTogglePress = true;

  //Driver Y
  bool hoodForwardBtn = false;
  //Driver X
  bool hoodBackBtn = false;

  float position = 0;

  void TeleopPeriodic() override
  {
    driveShooter.mLimeLight.tx = driveShooter.mLimeLight.table->GetNumber("tx", 0.0);
    driveShooter.mLimeLight.ty = driveShooter.mLimeLight.table->GetNumber("ty", 0.0);
    driveShooter.mLimeLight.ta = driveShooter.mLimeLight.table->GetNumber("ta", 0.0);
    driveShooter.mLimeLight.tv = driveShooter.mLimeLight.table->GetNumber("tv", 0.0);

    //std::cout <<"tv " << driveShooter.mLimeLight.tv << std::endl;

    driveRTrigger = driverController.GetRawAxis(3); //added on 3/19 to program shooter to trigger
    double limelightHasTarget = driveShooter.mLimeLight.table->GetNumber("tv", 0.0);

    driveShooter.printPIDFValues();
    driveShooter.setPIDFValues(false);

    driveShooter.updateLimelight(driveShooter.mLimeLight.ty, limelightHasTarget);
    driveShooter.updateButtons();
    driveIntake.updateBreakBeams(breakBeamNewBall.Get(), breakBeamIndex.Get(), breakBeamConveyorStart.Get(), breakBeamFull.Get());
    driveIntake.updateButtons();
    isTogglingIntake = driveIntake.driverController.GetRawButton(1);

    //make a button to reset encoder values
    DriveWithJoystick(false); //true = field relative, false = not field relative
    frc::SmartDashboard::PutNumber("fr encoder", frAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("fl encoder", flAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("bl encoder", blAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("br encoder", brAnalog.GetVoltage());

    //driveIntake.Run(); //commented out temporarily to test swerve

    //std::cout << "index beam" << breakBeamIndex.Get() << std::endl;
    //std::cout << "New Ball Sensor " << breakBeamConveyorStart.Get() << std::endl;
    //std::cout << "Intake Full Sensor " << breakBeamFull.Get() << std::endl;

    /*if (isTogglingIntake) //commented out temporarily to test swerve
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
    }*/
    frc::SmartDashboard::PutNumber("fl motor encoder", m_swerve.m_frontLeft.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("fr motor encoder", m_swerve.m_frontRight.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("bl motor encoder", m_swerve.m_backLeft.m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("br motor encoder", m_swerve.m_backRight.m_turningEncoder.GetPosition());

    driveShooter.Shoot();
  }

private:
  frc::Joystick driverController{0};
  frc::Joystick operatorController{1};

  //joystick defines
  double driveLeftX{driverController.GetRawAxis(0)};
  double driveLeftY{driverController.GetRawAxis(1)};
  double driveRightX{driverController.GetRawAxis(4)}; //4 on gamepad

  double driveLTrigger{driverController.GetRawAxis(2)};
  double driveRTrigger{driverController.GetRawAxis(3)};
  bool driveLBumper{driverController.GetRawButton(5)};
  bool driveRBumper{driverController.GetRawButton(6)};

  bool operatorBtnA{operatorController.GetRawButton(2)};

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
    driveLBumper = driverController.GetRawButton(5);
    driveLTrigger = driverController.GetRawAxis(2);

    driveLeftX = driverController.GetRawAxis(0);
    driveLeftY = driverController.GetRawAxis(1);
    driveRightX = -driverController.GetRawAxis(4);

    if (driveLeftY < deadzone && driveLeftY > (deadzone * -1) && driveLeftX < deadzone && driveLeftX > (deadzone * -1))
    {
      driveLeftY = 0;
      driveLeftX = 0;
    }

    if (driveRightX < deadzone && driveRightX > (deadzone * -1))
    {
      driveRightX = 0;
    }

    if (driveLTrigger > 0)
    {
      IntakeMotor.Set(ControlMode::PercentOutput, -driveLTrigger);
    }
    else
    {
      IntakeMotor.Set(0);
    }

    /*if (driveLBumper == true && resetEncoder == false)
    {
      resetTurnEncoder();
      resetEncoder = true;
    }
    else if (driveLBumper == false && resetEncoder == true)
    {
      resetEncoder = false;
    }*/

    /*if (driveLeftX == 0 && driveLeftY == 0 && driveRightX == 0)
    {
      m_swerve.m_frontLeft.m_turningEncoder.SetPosition(flturnStay);
      m_swerve.m_frontRight.m_turningEncoder.SetPosition(frturnStay);
      m_swerve.m_backLeft.m_turningEncoder.SetPosition(blturnStay);
      m_swerve.m_backRight.m_turningEncoder.SetPosition(brturnStay);
    }*/
    //put something different in if statement

    auto xSpeed = /*strafe*/ m_xspeedLimiter.Calculate(driveLeftX) * Drivetrain::kMaxSpeed;

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = /*-forward*/ -m_yspeedLimiter.Calculate(driveLeftY) * Drivetrain::kMaxSpeed;
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.to<double>());
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left

    frc::SmartDashboard::PutNumber("doubleGyro", m_swerve.doubleGyro());

    /*if (driveRightX == 0)
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

      auto rotStay = m_rotLimiter.Calculate(/*(rotError / 180) * kProt driveRightX) * Drivetrain::kMaxAngularSpeed;
      m_swerve.Drive(xSpeed, ySpeed, rotStay, fieldRelative, false, 0, 0); //field relative = true
    }
    else
    {*/
    auto rot = m_rotLimiter.Calculate(driveRightX) * Drivetrain::kMaxAngularSpeed;
    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true

    rotSetpoint = m_swerve.doubleGyro() /* + rotationCounter * 360*/; //maybe this should just be m_swerve.doubleGyro();
    //}

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, false, 0, 0); //field relative = true
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
    float forward = driveLeftY * cos(gyroRadians) + driveLeftX * sin(gyroRadians);
    float strafe = -driveLeftY * sin(gyroRadians) + driveLeftX * cos(gyroRadians);*/

    auto xSpeed = /*strafem_xspeedLimiter.Calculate(driveLeftX) * Drivetrain::kMaxSpeed;*/ cos(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;

    //frc::SmartDashboard::PutNumber("xSpeed", xSpeed.to<double>());
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = /*-forward-m_yspeedLimiter.Calculate(driveLeftY) * Drivetrain::kMaxSpeed*/ sin(direction * wpi::math::pi / 180) * Drivetrain::kMaxSpeed;

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

  void LimelightAim()
  {
    operatorBtnA = operatorController.GetRawButton(2);

    /*if (operatorBtnA)
    {
      
    }*/
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
    m_swerve.m_frontLeft.m_turningMotor.Disable();
    m_swerve.m_frontRight.m_turningMotor.Disable();
    m_swerve.m_backLeft.m_turningMotor.Disable();
    m_swerve.m_backRight.m_turningMotor.Disable();
  
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
    frc::Wait(0.2);
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}

#endif