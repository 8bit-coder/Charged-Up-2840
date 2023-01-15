// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMTalonSRX.h>

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDriveFront.SetExpiration(100_ms);
    m_robotDriveRear.SetExpiration(100_ms);
    m_timer.Start();
  }

  void AutonomousInit() override {
  }

  void AutonomousPeriodic() override {
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    double speed = -m_controller.GetRawAxis(1);
    double turn = -m_controller.GetRawAxis(0);
    m_robotDriveFront.ArcadeDrive(speed, turn);
    m_robotDriveRear.ArcadeDrive(speed, turn);
  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::PWMTalonSRX m_left{0};
  frc::PWMTalonSRX m_right{1};
  frc::PWMTalonSRX m_left2{2};
  frc::PWMTalonSRX m_right2{3};
  frc::DifferentialDrive m_robotDriveFront{m_left, m_right};
  frc::DifferentialDrive m_robotDriveRear{m_left2, m_right2};

  frc::XboxController m_controller{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
