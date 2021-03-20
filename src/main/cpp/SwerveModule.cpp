/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveModule.h"
#include <stdio.h>

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>

#include "Drivetrain.h"


double kP = 0.5 /*1.3*/, kI = 1e-6, kD = 0.0001, kIz = 0.001, kFF = 0 /*0.000015*/, kMaxOutput = 1.0, kMinOutput = -1.0;
double dkP = 0.0003, dkI = 1e-6, dkD = 0.00003, dkIz = 0.001, dkFF = 0.000156, dkMaxOutput = 1.0, dkMinOutput = -1.0;

double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
const double MaxRPM = 5700;

SwerveModule::SwerveModule(const int driveMotorChannel, const int turningMotorChannel, double gyroStartAng) : m_driveMotor{driveMotorChannel, brushless}, m_turningMotor{turningMotorChannel, brushless}
{
    frc::AnalogInput m_analog{turningMotorChannel/2 -1};

    m_turningMotor.RestoreFactoryDefaults();
    m_driveMotor.RestoreFactoryDefaults();
    //m_driveEncoder.SetPosition(0);

    double relativeEncoder = 0;

    //m_turnPIDController.SetFeedbackDevice(m_turningMotor.GetAnalog());
    
    if (m_analog.GetVoltage() <= 2.45) 
      relativeEncoder = (-9/2.45) * m_analog.GetVoltage();
    else if(m_analog.GetVoltage() > 2.45) 
      relativeEncoder = (-9/2.45) * m_analog.GetVoltage() + 18;

    std::cout << "relative encoder: " << relativeEncoder << "  m_analog: " << m_analog.GetVoltage() << std::endl;
    
    //m_turningEncoder.SetPosition(relativeEncoder);
    //double start_position = gyroStartAng + 180 / 180 * 9;

    //m_turningEncoder.SetPosition(start_position);

    frc::Wait(.002);

    //m_turningMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, .1);
    //m_turningMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, .1);
    
    //frc::SmartDashboard::PutNumber("Drive kIz", dkIz);
    //frc::SmartDashboard::PutNumber("Drive kFF", dkFF);

    

    m_drivePIDController.SetP(dkP);
    m_drivePIDController.SetI(dkI); //dkI
    m_drivePIDController.SetD(dkD); //dkD
    m_drivePIDController.SetIZone(dkIz); //dkIz
    m_drivePIDController.SetFF(dkFF); //dkFF
    m_drivePIDController.SetOutputRange(kMinOutput, kMaxOutput);

    /*frc::SmartDashboard::PutNumber("Turn P", kP);
    frc::SmartDashboard::PutNumber("Turn I", kI);
    frc::SmartDashboard::PutNumber("Turn D", kD);
    frc::SmartDashboard::PutNumber("Turn kIz", kIz);
    frc::SmartDashboard::PutNumber("Turn kFF", kFF);*/

    m_turnPIDController.SetP(frc::SmartDashboard::GetNumber("Turn P", kP));
    m_turnPIDController.SetI(frc::SmartDashboard::GetNumber("Turn I", kI));
    m_turnPIDController.SetD(frc::SmartDashboard::GetNumber("Turn D", 0));
    m_turnPIDController.SetIZone(frc::SmartDashboard::GetNumber("Turn kIz", kIz));
    m_turnPIDController.SetFF(frc::SmartDashboard::GetNumber("Turn kFF", 0));
    m_turnPIDController.SetOutputRange(-9, 9); //used to be -pi and pi

    m_drivePIDController.SetSmartMotionMaxVelocity(kMaxVel);
    m_drivePIDController.SetSmartMotionMinOutputVelocity(kMinVel);
    m_drivePIDController.SetSmartMotionMaxAccel(kMaxAcc);
    m_drivePIDController.SetSmartMotionAllowedClosedLoopError(kAllErr);
}

//for the following function we need to get the velocity of the drive encoder and the rotations of the turning motor
frc::SwerveModuleState SwerveModule::GetState()
{
    return {units::meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

/* double PrintSetAngle(const frc::SwerveModuleState &state)
{
    auto angle = state.angle.Radians();
    double doubleangle = angle.to<double>();
    return doubleangle * 100;
} */

bool SwerveModule::SetDesiredState(const frc::SwerveModuleState &state, bool autonomous, double distance, double maxVelocity)
{
    // Calculate the drive output from the drive PID controller.

    // Calculate the turning motor output from the turning PID controller.
    auto angle = state.angle.Radians();
    double doubleangle = angle.to<double>();
    double command = (doubleangle / wpi::math::pi)* 9;
    double stateSpeed = state.speed.to<double>();
    double current = m_turningEncoder.GetPosition(); 

    // Set the motor outputs.
    if (!autonomous) {
        m_drivePIDController.SetReference(stateSpeed * speedFlip * 435, rev::ControlType::kVelocity); //*3000
        //std::cout << "speed reference" <<stateSpeed * speedFlip * 435 << std::endl;
    } else {
        //kMaxVel = maxVelocity;
        m_drivePIDController.SetReference(distance * 8.333333333/(wpi::math::pi * 4) / 1.075 /* speedFlip*/, rev::ControlType::kSmartMotion); ///1.075 because real life is disappointing
        //std::cout << "encoder value: " << m_driveEncoder.GetPosition() << "  distance: " << distance * 8.333333333/(wpi::math::pi * 4) << std::endl;
        m_error = m_driveEncoder.GetPosition() - distance * 8.333333333/(wpi::math::pi * 4) / 1.075 *speedFlip;
    }
    
    frc::SmartDashboard::PutNumber("Drive Speed", state.speed.to<double>() * 500);

    //this plus speed flip causes the robot to spin non stop
    frc::SmartDashboard::PutNumber("Current - Command", current - command);
    if (((current - command) >= 9)) 
    {
        while((current - command) >= 9) {
        command += 18;
        }
    } else {
         while((current - command) < -9) {
             command -= 18;
         }
    } 
    

    if (abs(current - command) >= 4.5 && flipping == 0) {    //changed the inequality signs to <,> from =<,=>
            m_turningEncoder.SetPosition(current + 9);
            speedFlip *= -1;
            flipping = 1;
       //std::cout << "flipped  " << "flipping: " << flipping << std::endl;
    } else if (abs(current - command) < 4.5 && flipping == 1) {
        flipping = 0;
      //std::cout << "flipped  " << "flipping: " << flipping << std::endl;
    }

    frc::SmartDashboard::PutNumber("Drive P", dkP);
    //frc::SmartDashboard::PutNumber("Drive I", dkI);
    frc::SmartDashboard::PutNumber("Drive D", dkD);

    if (dkP != frc::SmartDashboard::GetNumber("Drive P", dkP)) {
        dkP = frc::SmartDashboard::GetNumber("Drive P", dkP);
    }

    if (dkD != frc::SmartDashboard::GetNumber("Drive D", dkD)) {
        dkD = frc::SmartDashboard::GetNumber("Drive D", dkD);
    }

    frc::SmartDashboard::PutNumber("encoder command", m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("speed flip", speedFlip);
    
    m_turnPIDController.SetReference(command, rev::ControlType::kPosition);
    //this->lastCommand = command - (18*(lastCommand/abs(lastCommand)));
    frc::SmartDashboard::PutNumber("Encoder Position", m_turningEncoder.GetPosition() * 9);
    frc::SmartDashboard::PutNumber("Turn Set Reference", (doubleangle / wpi::math::pi) * 9);
    frc::SmartDashboard::PutNumber("Rotate Speed", doubleangle);
    frc::SmartDashboard::PutNumber("command", command);

    //std::cout << "error: " << m_error << std::endl;

    if (autonomous == 1) {
        if (abs(m_error) < 1) {
            //m_driveEncoder.SetPosition(0);
            return true;
        } 
        return false;
    }
    return false;
}

void SwerveModule::zeroDriveEncoder() {
    m_driveEncoder.SetPosition(0);
}

void SwerveModule::zeroTurnEncoder()
{
    rev::CANAnalog m_analogSensor = m_turningMotor.GetAnalog(); //get the analog
    //home position of analog encoder is 2.45
    //get the difference between the home pos and the current pos on startup
    double dist_to_home = m_analogSensor.GetVoltage() - 2.45;

}

