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
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering and an Xbox controller.
 */
class Robot : public frc::TimedRobot {
  //Sparkmax Motor Objects Created
  rev::CANSparkMax m_leftMotor1{1, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_rightMotor1{3,  rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_leftMotor2{2, rev::CANSparkLowLevel::MotorType::kBrushless };
  rev::CANSparkMax m_rightMotor2{4,  rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_armMotor{5,  rev::CANSparkLowLevel::MotorType::kBrushless};

  //Encoder objects created, mapped to existing Sparkmax Objects
  rev::SparkRelativeEncoder m_encoder1 = m_leftMotor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoder2 = m_leftMotor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoder3 = m_rightMotor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoder4 = m_rightMotor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder armEncoder = m_armMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  

  //Creating PIDs for encoders
  //rev::SparkPIDController m_pidController1 = m_leftMotor1.GetPIDController();
  rev::SparkPIDController m_pidController2 = m_leftMotor2.GetPIDController();
  rev::SparkPIDController m_pidController3  = m_rightMotor1.GetPIDController();
  rev::SparkPIDController m_pidController4 = m_rightMotor2.GetPIDController();

  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;


  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_leftMotor1.Set(output); },
      [&](double output) { m_rightMotor1.Set(output); }};
  frc::XboxController m_driverController{0};

  private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Nothing";
  const std::string kauto1 = "Auto 1";
  std::string m_autoSelected;
  frc::Timer m_autoTimer;

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

    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor1);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor1);


    m_leftMotor1.SetInverted(true);
    m_armMotor.SetInverted(true);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor2.Follow(m_leftMotor1);
    m_rightMotor2.Follow(m_rightMotor1);    
    armEncoder.SetPosition(0);

    //Initialize Autonomous Robot Options
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kauto1, kauto1);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

void AutonomousInit() override {
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto Selected: {}\n", m_autoSelected);
  m_autoTimer.Reset();
  m_autoTimer.Start();
}

void AutonomousPeriodic() override {
  int newArmPos;
  m_robotDrive.ArcadeDrive(0,0);
  fmt::print("Do Nothing");

  if (m_autoSelected == kauto1) {
    fmt::print("Nothing to see here");
    // Custom Auto goes here
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 1_s) {
      newArmPos = 30; //Rotate Arm
      if (armEncoder.GetPosition() < 75) {
        m_armMotor.Set(0.5);
      }
      else {
        m_armMotor.Set(0);
      }
   }
  } else {
      // Default Auto goes here
      fmt::print("The adverse effect");
  }
}

  void TeleopPeriodic() override {
    // Drive with tank style
    m_robotDrive.ArcadeDrive(-m_driverController.GetLeftY(),
                           m_driverController.GetLeftX());
    
    if (armEncoder.GetPosition() < 75) {
      m_armMotor.Set(0.5);
    }
    else {
      m_armMotor.Set(0);
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
    double pos = armEncoder.GetPosition();
    std::cout << pos;
    
    frc::SmartDashboard::PutNumber("Set Rotations",pos);
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
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
