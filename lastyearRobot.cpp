/ Include Header File:
#include "Robot.h"

// Initialize Default Robot Variables:
void Robot::RobotInit()
{
  // Camera Stream:
  #if defined(__linux__)
  //frc::CameraServer::GetInstance()->StartAutomaticCapture();
  #else
  wpi::errs() << "Vision only available on Linux.\n";
  wpi::errs().flush();
  #endif

  // PDB Reset Sticky Faults & Total Energy:
  PDB.ClearStickyFaults();
  PDB.ResetTotalEnergy();

  // Initialize Autonomous Mode Options:
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kautoBumperHigh, kautoBumperHigh);
  m_chooser.AddOption(kautoBumperLow, kautoBumperLow);
  m_chooser.AddOption(kautoTarmac, kautoTarmac);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Invert Left Drive Motors:
  m_frontLeft.SetInverted(true);
  m_rearLeft.SetInverted(true);

  // Turn Puncher Piston Off:
  PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
  // Turn Hanger Pistons Forward:
  hangerRight.Set(frc::DoubleSolenoid::kForward);
  hangerLeft.Set(frc::DoubleSolenoid::kForward);

  // Set Arm Encoders To 0:
  m_rightArmEncoder.SetPosition(0);
  m_leftArmEncoder.SetPosition(0);

  // Set All PID Coeffecients:
  setPID();
}

// Initialize Autonomous Mode:
void Robot::AutonomousInit()
{
  // Set Auto Mode To Text Selected In Smart Dashboard:
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  // Start Timer:
  m_autoTimer.Reset();
  m_autoTimer.Start();
}

// Autonomous Periodic:
void Robot::AutonomousPeriodic()
{
  // High Goal Shot Autonomous (From Tarmac):
  if (m_autoSelected == kautoBumperHigh)
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 2_s)
    {
      // Set PID Loop, Release Pistons:
      diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
      diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
      m_rightArmPIDController.SetReference(bumperPos, rev::CANSparkMax::ControlType::kPosition);
      m_leftArmPIDController.SetReference(-bumperPos, rev::CANSparkMax::ControlType::kPosition);

      m_robotDrive.ArcadeDrive(0, 0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(1800, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-1800, rev::CANSparkMax::ControlType::kVelocity);
    }
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 6_s)
    {
      m_rightArm.Set(0);
      m_leftArm.Set(0);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
      diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
      m_robotDrive.ArcadeDrive(0.6, 0);
    }
    else if (m_autoTimer.Get() > 6_s)
    {
      m_robotDrive.ArcadeDrive(0, 0);
      m_autoTimer.Stop();
      m_autoTimer.Reset();
    }
  } 

  // Low Goal Shot Autonomous (From Tarmac):
  else if (m_autoSelected == kautoBumperLow)
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 2_s)
    { 
      m_robotDrive.ArcadeDrive(0, 0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(900, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-900, rev::CANSparkMax::ControlType::kVelocity);
    }
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 6_s)
    {
      m_rightArm.Set(0);
      m_leftArm.Set(0);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
      diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
      m_robotDrive.ArcadeDrive(0.6, 0);
    }
    else if (m_autoTimer.Get() > 6_s)
    {
      m_robotDrive.ArcadeDrive(0, 0);

      m_autoTimer.Stop();
      m_autoTimer.Reset();
    }
  }

  // High Goal Shot Autonomous (From Outside Tarmac):
  else if (m_autoSelected == kautoTarmac)
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 2_s)
    {
      m_robotDrive.ArcadeDrive(0.6, 0);
    }
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0, 0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(2200, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-2200, rev::CANSparkMax::ControlType::kVelocity);
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 6_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    }
    else if (m_autoTimer.Get() > 6_s && m_autoTimer.Get() < 7_s)
    {
      m_rightArm.Set(0);
      m_leftArm.Set(0);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
      diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
    }
    else if (m_autoTimer.Get() > 7_s)
    {
      m_autoTimer.Stop();
      m_autoTimer.Reset();
    }
  }

  // If No Auto Mode Is Selected:
  else
  {
     diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
     diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
     m_robotDrive.ArcadeDrive(0, 0);
     m_autoTimer.Reset();
     m_autoTimer.Stop();
  }
}

// Teleop Periodic:
void Robot::TeleopPeriodic()
{
  // Arcade Drive (X Direction Sensitivity Decreased & Inverted):
  m_robotDrive.ArcadeDrive(m_controller.GetLeftTriggerAxis() - m_controller.GetRightTriggerAxis(), (-m_controller.GetLeftX() * 0.8));

  // Arm Controlls:-------------------------------------------------------------------------------

   // Use PID To Get Arms To Bumper Shots Position:
   if (m_controller.GetAButton())
   {
     m_rightShoot.Set(0);
     m_leftShoot.Set(0);
     diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
     diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
     m_rightArmPIDController.SetReference(bumperPos, rev::CANSparkMax::ControlType::kPosition);
     m_leftArmPIDController.SetReference(-bumperPos, rev::CANSparkMax::ControlType::kPosition);
     bumperShot = true;
   }
   else if ((m_rightArmEncoder.GetPosition() >= bumperPos || m_leftArmEncoder.GetPosition() <= -bumperPos) && bumperShot)
   {
     diskBrakeRight.Set(frc::DoubleSolenoid::kReverse);
     diskBrakeLeft.Set(frc::DoubleSolenoid::kReverse);
     m_rightArm.Set(0);
     m_leftArm.Set(0);
     bumperShot = false;
   }

   // Use PID To Get Arms To Tarmac Shot Position:
   if (m_controller.GetXButton())
   {
     m_rightShoot.Set(0);
     m_leftShoot.Set(0);
     diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
     diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
     m_rightArmPIDController.SetReference(tarmacPos, rev::CANSparkMax::ControlType::kPosition);
     m_leftArmPIDController.SetReference(-tarmacPos, rev::CANSparkMax::ControlType::kPosition);
     tarmacShot = true;
   }
   else if ((m_rightArmEncoder.GetPosition() >= tarmacPos || m_leftArmEncoder.GetPosition() <= -tarmacPos) && tarmacShot)
   {
     diskBrakeRight.Set(frc::DoubleSolenoid::kReverse);
     diskBrakeLeft.Set(frc::DoubleSolenoid::kReverse);
     m_rightArm.Set(0);
     m_leftArm.Set(0);
     tarmacShot = false;
   }

   // Let Arms Down:
   if (m_controller.GetBButton())
  {
     diskBrakeRight.Set(frc::DoubleSolenoid::kForward);
     diskBrakeLeft.Set(frc::DoubleSolenoid::kForward);
  }

  // If Limit Switches Pressed, Set Arm Positions To 0, armRaised False, & Disk Brakes Forward:
  if (!rightLimit.Get())
  {
    m_rightArmEncoder.SetPosition(0);
    m_leftArmEncoder.SetPosition(0);
  }
  if (!leftLimit.Get())
  {
    m_rightArmEncoder.SetPosition(0);
    m_leftArmEncoder.SetPosition(0);
  }

  // ---------------------------------------------------------------------------------------------

  // Ball Shooter Controlls:----------------------------------------------------------------------

  // Ball Output Routine For Low Goal (From Bumper With Timer):
  if (m_controller.GetBackButton())
  {
    m_lowGoalTimer.Reset();
    m_lowGoalTimer.Start();
  }
  else
  {
    if (m_lowGoalTimer.Get() > 0_s && m_lowGoalTimer.Get() < 2_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(900, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-900, rev::CANSparkMax::ControlType::kVelocity);
    }
    if (m_lowGoalTimer.Get() > 2_s && m_lowGoalTimer.Get() < 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    } 
    if (m_lowGoalTimer.Get() > 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      m_lowGoalTimer.Stop();
      m_lowGoalTimer.Reset();
    }
  }

  // Ball Output For High Goal (From Bumper With Timer):
  if (m_controller.GetStartButton())
  {
    m_highGoalTimer.Reset();
    m_highGoalTimer.Start();
  }
  else
  {
    if (m_highGoalTimer.Get() > 0_s && m_highGoalTimer.Get() < 2_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(2500, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-2500, rev::CANSparkMax::ControlType::kVelocity);
    }
    if (m_highGoalTimer.Get() > 2_s && m_highGoalTimer.Get() < 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    }
    if (m_highGoalTimer.Get() > 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      m_highGoalTimer.Stop();
      m_highGoalTimer.Reset();
    }
  }

  // Ball Output For High Goal (From Tarmac With Timer):
  if (m_controller.GetYButton())
  {
    m_tarmacTimer.Reset();
    m_tarmacTimer.Start();
  }
  else
  {
    if (m_tarmacTimer.Get() > 0_s && m_tarmacTimer.Get() < 2_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShootPIDController.SetReference(4200, rev::CANSparkMax::ControlType::kVelocity);
      m_leftShootPIDController.SetReference(-4200, rev::CANSparkMax::ControlType::kVelocity);
    }
    if (m_tarmacTimer.Get() > 2_s && m_tarmacTimer.Get() < 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kForward);
    }
    if (m_tarmacTimer.Get() > 2.5_s)
    {
      PuncherDouble.Set(frc::DoubleSolenoid::kReverse);
      m_rightShoot.Set(0);
      m_leftShoot.Set(0);
      m_tarmacTimer.Stop();
      m_tarmacTimer.Reset();
    }
  }

  // ---------------------------------------------------------------------------------------------

  // Shooter Intake & Outtake:
  if (m_controller.GetLeftBumper())
  {
    m_rightShootPIDController.SetReference(-800, rev::CANSparkMax::ControlType::kVelocity);
    m_leftShootPIDController.SetReference(800, rev::CANSparkMax::ControlType::kVelocity);
  }
  else if (m_controller.GetRightBumper())
  {
    m_rightShootPIDController.SetReference(800, rev::CANSparkMax::ControlType::kVelocity);
    m_leftShootPIDController.SetReference(-800, rev::CANSparkMax::ControlType::kVelocity);
  }
  else if (m_controller.GetLeftStickButton() || m_controller.GetRightStickButton())
  {
    m_rightShoot.Set(0);
    m_leftShoot.Set(0);
  }
}

// Set All PID Coeffecients Function:
void Robot::setPID()
{
  // Set Right Arm Motor PID Coeffecients:
  m_rightArmPIDController.SetP(AkP);
  m_rightArmPIDController.SetI(AkI);
  m_rightArmPIDController.SetD(AkD);
  m_rightArmPIDController.SetIZone(AkIz);
  m_rightArmPIDController.SetFF(AkFF);
  m_rightArmPIDController.SetOutputRange(AkMinOutput, AkMaxOutput);
  // Set Left Arm Motor PID Coeffecients:
  m_leftArmPIDController.SetP(AkP);
  m_leftArmPIDController.SetI(AkI);
  m_leftArmPIDController.SetD(AkD);
  m_leftArmPIDController.SetIZone(AkIz);
  m_leftArmPIDController.SetFF(AkFF);
  m_leftArmPIDController.SetOutputRange(AkMinOutput, AkMaxOutput);

  // Set Right Shooter Motor PID Coeffecients:
  m_rightShootPIDController.SetP(SkP);
  m_rightShootPIDController.SetI(SkI);
  m_rightShootPIDController.SetD(SkD);
  m_rightShootPIDController.SetIZone(SkIz);
  m_rightShootPIDController.SetFF(SkFF);
  m_rightShootPIDController.SetOutputRange(SkMinOutput, SkMaxOutput);
  // Set Left Shooter Motor PID Coeffecients:
  m_leftShootPIDController.SetP(SkP);
  m_leftShootPIDController.SetI(SkI);
  m_leftShootPIDController.SetD(SkD);
  m_leftShootPIDController.SetIZone(SkIz);
  m_leftShootPIDController.SetFF(SkFF);
  m_leftShootPIDController.SetOutputRange(SkMinOutput, SkMaxOutput);
}

// Unused Robot Functions:
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}

void Robot::TestPeriodic() 
{
  m_robotDrive.ArcadeDrive((m_controller.GetLeftTriggerAxis() - m_controller.GetRightTriggerAxis()) * 0.5, -m_controller.GetLeftX() * 0.5);
}

void Robot::TeleopInit() {}
void Robot::RobotPeriodic() {} // Function Called Every Robot Packet, No Matter The Mode (Good For Diagnostics):

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif