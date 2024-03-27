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

  // rev::CANSparkMax m_rightMotorFront{1, rev::CANSparkLowLevel::MotorType::kBrushless };
  // rev::CANSparkMax m_leftMotorFront{3,  rev::CANSparkLowLevel::MotorType::kBrushless };
  // rev::CANSparkMax m_rightMotorBack{2, rev::CANSparkLowLevel::MotorType::kBrushless };
  // rev::CANSparkMax m_leftMotorBack{4,  rev::CANSparkLowLevel::MotorType::kBrushless};
  // rev::CANSparkMax m_leftArmMotor{5,  rev::CANSparkLowLevel::MotorType::kBrushless};
  // rev::CANSparkMax m_rightArmMotor{6,  rev::CANSparkLowLevel::MotorType::kBrushless};
  // rev::CANSparkMax m_topShooterMotor{7,  rev::CANSparkLowLevel::MotorType::kBrushless};
  // rev::CANSparkMax m_bottomShooterMotor{8,  rev::CANSparkLowLevel::MotorType::kBrushless};
  // rev::CANSparkMax m_intakeMotor{9,  rev::CANSparkLowLevel::MotorType::kBrushless};
  //Sparkmax Motor Objects Created
  rev::CANSparkMax m_rightMotorFront{2, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_leftMotorFront{3,  rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_rightMotorBack{1, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_leftMotorBack{4,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftArmMotor{5,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightArmMotor{6,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_topShooterMotor{7,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_bottomShooterMotor{8,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_intakeMotor{9,  rev::CANSparkLowLevel::MotorType::kBrushless};


  //Encoder objects created, mapped to existing Sparkmax Objects
  rev::SparkRelativeEncoder m_encoderR1 = m_rightMotorFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderR2 = m_rightMotorBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderR3 = m_leftMotorFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderR4 = m_leftMotorBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_RarmEncoder = m_rightArmMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_LarmEncoder = m_leftArmMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_shooterEncoderT = m_topShooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_shooterEncoderB = m_bottomShooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  

  //Creating PIDs for encoders
  //rev::SparkPIDController m_pidController1 = m_rightMotorFront.GetPIDController();
  rev::SparkPIDController m_pidController2 = m_rightMotorBack.GetPIDController();
  rev::SparkPIDController m_pidController3  = m_leftMotorFront.GetPIDController();
  rev::SparkPIDController m_pidController4 = m_leftMotorBack.GetPIDController();

  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  bool ba_intake = false, bx_sShooter = false, by_aShooter = false, b_forward = false, b_reverse = false, b_up = false, b_down = false, bb_outtake = false;


  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_rightMotorFront.Set(output); },
      [&](double output) { m_leftMotorFront.Set(output); }};
  frc::XboxController m_driverController{0};

  private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Nothing";
  const std::string kauto1 = "Auto 1";
  std::string m_autoSelected;
  frc::Timer m_autoTimer;
  // Right And Left Arm Limit Switches:
  frc::DigitalInput rightLimit{9};
  frc::DigitalInput leftLimit{8};
 public:
  void RobotInit() override {
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANEncoder object. 
     */
    //m_pidController1.SetFeedbackDevice(m_encoder1);

    // set PID coefficients
    //m_pidController1.SetP(kP);
    //m_pidController1.SetI(kI);
    //m_pidController1.SetD(kD);
    //m_pidController1.SetIZone(kIz);
    //m_pidController1.SetFF(kFF);
    //m_pidController1.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotorFront);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotorFront);


    // m_rightMotorFront.SetInverted(true);
    m_leftMotorFront.SetInverted(true);
    m_bottomShooterMotor.SetInverted(true);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotorBack.Follow(m_rightMotorFront);
    m_leftMotorBack.Follow(m_leftMotorFront);
    // m_armEncoder.SetPosition(0);

    
}


  void TeleopPeriodic() override {
    ba_intake = m_driverController.GetAButton();
    bb_outtake = m_driverController.GetBButton();
    bx_sShooter = m_driverController.GetXButton();
    by_aShooter = m_driverController.GetYButton();
    b_forward = m_driverController.GetRightTriggerAxis();
    b_reverse = m_driverController.GetLeftTriggerAxis();
    b_up = m_driverController.GetRightBumper();
    b_down = m_driverController.GetLeftBumper();
    // Drive with tank style
    m_robotDrive.ArcadeDrive(m_driverController.GetRightY(),
                            -m_driverController.GetRightX());

    //forward
    // if (b_forward){
    //   m_rightMotorFront.Set(0.5);
    //   m_leftMotorFront.Set(0.5);
    // }
    // else if (b_reverse){
    //   m_rightMotorFront.Set(-0.5);
    //   m_leftMotorFront.Set(-0.5);
    // }
    // else {
    //   m_rightMotorFront.Set(0);
    //   m_leftMotorFront.Set(0);
    // }
    
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
    // if (bx_sShooter) {
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
    //speaker_shooter
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

    //armup
    if (b_up) {
      m_leftArmMotor.Set(-0.5);
      m_rightArmMotor.Set(-0.5);
    }
    else if (b_down) {
      m_leftArmMotor.Set(0.5);
      m_rightArmMotor.Set(0.5);
    }
    else { 
      m_leftArmMotor.Set(0);
      m_rightArmMotor.Set(0);
    }
    


    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
    double rPos = m_RarmEncoder.GetPosition();
    double lPos = m_LarmEncoder.GetPosition();
    frc::SmartDashboard::PutNumber("Right Encoder Output", rPos);
    frc::SmartDashboard::PutNumber("Left Encoder Output", lPos);
    
    std::cout << lPos;
    
    // frc::SmartDashboard::PutNumber("Set Rotations",pos);
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    //if((p != kP)) { m_pidController1.SetP(p); kP = p; }
    // if((i != kI)) { m_pidController1.SetI(i); kI = i; }
    // if((d != kD)) { m_pidController1.SetD(d); kD = d; }
    // if((iz != kIz)) { m_pidController1.SetIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController1.SetFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //  m_pidController1.SetOutputRange(min, max); 
    //  kMinOutput = min; kMaxOutput = max;
  //}
  /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  rev::CANSparkMax::ControlType::kDutyCycle
     *  rev::CANSparkMax::ControlType::kPosition
     *  rev::CANSparkMax::ControlType::kVelocity
     *  rev::CANSparkMax::ControlType::kVoltage
     */
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
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
