// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/voltage.h>
#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering and an Xbox controller.
 */
class Robot : public frc::TimedRobot {

  /*Sparkmax Motor Objects Created*/
  //Drive Train
  rev::CANSparkMax m_rightMotorFront{2, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_leftMotorFront{3,  rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_rightMotorBack{1, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_leftMotorBack{4,  rev::CANSparkLowLevel::MotorType::kBrushless};
  //Arm
  rev::CANSparkMax m_leftArmMotor{5,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightArmMotor{6,  rev::CANSparkLowLevel::MotorType::kBrushless};
  //Shooters
  rev::CANSparkMax m_topShooterMotor{7,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_bottomShooterMotor{8,  rev::CANSparkLowLevel::MotorType::kBrushless}; 
  //Intake
  rev::CANSparkMax m_intakeMotor{9,  rev::CANSparkLowLevel::MotorType::kBrushless};

  /*Encoder objects created, mapped to existing Sparkmax Objects */
  //Drive Train
  rev::SparkRelativeEncoder m_encoderR1 = m_rightMotorFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderR3 = m_leftMotorFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  //Arm
  rev::SparkRelativeEncoder m_RarmEncoder = m_rightArmMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_LarmEncoder = m_leftArmMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  //Shooters
  rev::SparkRelativeEncoder m_shooterEncoderT = m_topShooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  //Intake
  rev::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

  //Creating PIDs for encoders
  rev::SparkPIDController m_pidRightArmController = m_rightArmMotor.GetPIDController();
  rev::SparkPIDController m_pidLeftArmController  = m_leftArmMotor.GetPIDController();
  //Declaring Controller Variables
  bool ba_intake, bx_sShooter, by_aShooter, b_forward, b_reverse, b_up, b_down, bb_outtake = false;
  bool b_start, b_back;
  //Differential Drive Instantiation
  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_rightMotorFront.Set(output); },
      [&](double output) { m_leftMotorFront.Set(output); }};
  frc::XboxController m_driverController{0};

  // private:
  // frc::SendableChooser<std::string> m_chooser;
  // const std::string kAutoNameDefault = "Nothing";
  // const std::string kauto1 = "Auto 1";
  // std::string m_autoSelected;

  // Right And Left Arm Limit Switches:
  frc::DigitalInput rightLimit{9};
  frc::DigitalInput leftLimit{8};

  //Timer objects Created
  frc::Timer m_autoTimer;
  frc::Timer m_speakerTimer;
  frc::Timer m_ampTimer;
  bool b_speakerShot = false;
  bool b_ampShot = false;

 public:
  void RobotInit() override {
    // set PID coefficients
    m_pidRightArmController.SetP(0.10);
    m_pidLeftArmController.SetP(0.10);
    m_pidRightArmController.SetI(0.0);
    m_pidLeftArmController.SetI(0.0);
    m_pidRightArmController.SetD(0.01);
    m_pidLeftArmController.SetD(0.01);    
    m_pidRightArmController.SetIZone(0.0);
    m_pidLeftArmController.SetIZone(0.0);
    m_pidRightArmController.SetFF(0.0);
    m_pidLeftArmController.SetFF(0.0);
    m_pidRightArmController.SetOutputRange(-1.0,1.0);
    m_pidLeftArmController.SetOutputRange(-1.0,1.0);    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotorFront.SetInverted(true);
    m_leftMotorFront.SetInverted(true);
    m_bottomShooterMotor.SetInverted(true);
    //Setting back motors to follow parent motors
    m_rightMotorBack.Follow(m_rightMotorFront);
    m_leftMotorBack.Follow(m_leftMotorFront);
    // m_leftArmMotor.Follow(m_rightArmMotor);
    //Arm Encoder
    m_RarmEncoder.SetPosition(0);
    m_LarmEncoder.SetPosition(0);  
}
  // Teleop Periodic
  void TeleopPeriodic() override {
    ba_intake = m_driverController.GetAButton();
    bb_outtake = m_driverController.GetBButton();
    bx_sShooter = m_driverController.GetXButton();
    by_aShooter = m_driverController.GetYButton();
    b_forward = m_driverController.GetLeftTriggerAxis();
    b_reverse = m_driverController.GetRightTriggerAxis();
    b_up = m_driverController.GetRightBumper();
    b_down = m_driverController.GetLeftBumper();
    b_start = m_driverController.GetStartButton();
    b_back = m_driverController.GetBackButton();
    //Arcade Drive
    m_robotDrive.ArcadeDrive((b_forward - b_reverse),
                            -m_driverController.GetLeftX());
    //intake
    if (ba_intake) {
      m_intakeMotor.Set(0.5);
    }
    else if (bb_outtake) {
      m_intakeMotor.Set(-0.5); 
    }
    else {
      m_intakeMotor.Set(0);
    }
    //speaker_shooter
    if (bx_sShooter) {
       //m_pidRightArmController.SetReference(-4.0,  rev::CANSparkMax::ControlType::kPosition);
       //m_pidLeftArmController.SetReference(-4.0,  rev::CANSparkMax::ControlType::kPosition);
       m_topShooterMotor.Set(-0.5);
       m_bottomShooterMotor.Set(0.5);
     }
     else {
      //m_leftArmMotor.SetReference(0.0);
      // m_intakeMotor.Set(0.5);
       m_topShooterMotor.Set(0) ;
       m_bottomShooterMotor.Set(0);
     }
    //amp_shooter
    // if (by_aShooter) {
    //   m_leftArmMotor.Set(-0.5);
    //   m_rightArmMotor.Set(-0.5);
    //   m_topShooterMotor.Set(-0.5);
    //   m_bottomShooterMotor.Set(0.5);
    // }
    // else {
    //   // m_intakeMotor.Set(0.5);
    //   m_topShooterMotor.Set(0) ;
    //   m_bottomShooterMotor.Set(0);
    // }
    //Speaker Shooter Sequence
    if( b_start || b_speakerShot){
       if (!b_speakerShot){
        b_speakerShot = true;
        m_speakerTimer.Reset();
        m_speakerTimer.Start();
        m_pidRightArmController.SetReference(-5.5,  rev::CANSparkMax::ControlType::kPosition);
        // m_pidLeftArmController.SetReference(-5.5,  rev::CANSparkMax::ControlType::kPosition);
       }
       if (m_speakerTimer.Get() > 1_s && m_speakerTimer.Get() < 3_s){
         m_topShooterMotor.Set(-0.75);
         m_bottomShooterMotor.Set(0.75);
       }
       if (m_speakerTimer.Get() > 3_s && m_speakerTimer.Get() < 5_s){
         m_intakeMotor.Set(0.5);
       }
       if (m_speakerTimer.Get() > 5_s ){
         b_speakerShot = false;
       }
    }
//Amp Shooter Sequence
    if( b_back || b_ampShot){
       if (!b_ampShot){
        b_ampShot = true;
        m_ampTimer.Reset();
        m_ampTimer.Start();
        m_pidRightArmController.SetReference(-50.0,  rev::CANSparkMax::ControlType::kPosition);
        // m_pidLeftArmController.SetReference(-8.3,  rev::CANSparkMax::ControlType::kPosition);
       }
       if (m_ampTimer.Get() > 2_s && m_ampTimer.Get() < 3_s){
         m_topShooterMotor.Set(-0.5);
         m_bottomShooterMotor.Set(0.50);
         m_intakeMotor.Set(0.5);
       }
       if (m_ampTimer.Get() > 3_s && m_ampTimer.Get() < 5_s){
         m_pidRightArmController.SetReference(-2.0,  rev::CANSparkMax::ControlType::kPosition);
         m_intakeMotor.Set(0.0);
         m_topShooterMotor.Set(-0.0);
         m_bottomShooterMotor.Set(0.0);
       }
       if (m_ampTimer.Get() > 5_s ){
         b_ampShot = false;
       }
    }
    if (b_up && !b_speakerShot && !b_ampShot) {
        m_leftArmMotor.Set(-0.5);
        m_rightArmMotor.Set(-0.5);
      }
    else if (b_down && !b_speakerShot && !b_ampShot) {
        m_leftArmMotor.Set(0.25);
        m_rightArmMotor.Set(0.25);
    }
    else if(!b_speakerShot && !b_ampShot){ 
        m_leftArmMotor.Set(0);
        m_rightArmMotor.Set(0);
    }

    // m_ampTimer.Reset();
    // m_speakerTimer.Reset();

    // print data to SmartDashboard
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
    double rPos = m_RarmEncoder.GetPosition();
    double lPos = m_LarmEncoder.GetPosition();
    frc::SmartDashboard::PutNumber("Right Encoder Output", rPos);
    frc::SmartDashboard::PutNumber("Left Encoder Output", lPos);
    frc::SmartDashboard::PutNumber("Left Motor Output", m_leftArmMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Right Motor Output", m_rightArmMotor.GetOutputCurrent());
    
    std::cout << lPos;
     
    //m_pidController1.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
    
    //frc::SmartDashboard::PutNumber("SetPoint", rotations);
    //frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder1.GetPosition());
    
  }

  void RobotPeriodic() override{
    // If Limit Switches Pressed, Set Arm Positions To 0, armRaised False, & Disk Brakes Forward:
    //printf("rightarm down");
    //printf("test1 %f", m_RarmEncoder.GetPosition());
    //printf("test %f",m_LarmEncoder.GetPosition());

  if (!rightLimit.Get())
  {
    frc::SmartDashboard::PutString("RightArm Status", "rightarm down");
    printf("test1 %f", m_RarmEncoder.GetPosition());
    m_RarmEncoder.SetPosition(0);
    m_LarmEncoder.SetPosition(0);
  }
  else if(rightLimit.Get())
  {
    frc::SmartDashboard::PutString("RightArm Status", "rightarm up");
    printf("test2 %f", m_RarmEncoder.GetPosition());
  }

  if (!leftLimit.Get())
  {
    frc::SmartDashboard::PutString("LeftArm Status", "leftarm down");
    printf("test3 %f",m_LarmEncoder.GetPosition());
    m_RarmEncoder.SetPosition(0);
    m_LarmEncoder.SetPosition(0);
  }
  else if(leftLimit.Get())
  {
    frc::SmartDashboard::PutString("LeftArm Status", "leftarm up");
    printf("test4 %f", m_RarmEncoder.GetPosition());
  }
  fflush(stdout);
  fflush(stderr);
  }

void AutonomousInit() {
  // Set Auto Mode To Text Selected In Smart Dashboard:
  // m_autoSelected = m_chooser.GetSelected();
  // fmt::print("Auto selected: {}\n", m_autoSelected);
  // Start Timer:
  m_autoTimer.Reset();
  m_autoTimer.Start();
}

void AutonomousPeriodic() {
  m_autoTimer.Reset();
  m_autoTimer.Start();
  if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 3_s)
    {
      // Move drive train forward at 10% speed for 3 seconds
      m_leftMotorFront.Set(0.10);
      m_rightMotorFront.Set(0.10);
    }
    else {
      m_leftMotorFront.Set(0);
      m_rightMotorFront.Set(0);
    }
}
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
