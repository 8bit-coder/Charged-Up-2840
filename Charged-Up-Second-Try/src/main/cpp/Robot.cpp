#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/SerialPort.h>
#include <ctre/Phoenix.h>
#include <stdio.h>
#include <iostream>
//yike

frc::PWMTalonSRX m_left{0};
frc::PWMTalonSRX m_right{1};
frc::PWMTalonSRX m_left2{2};
frc::PWMTalonSRX m_right2{3};

ctre::phoenix::motorcontrol::can::TalonFX m_armMotor1(0);
frc::PWMTalonSRX m_armMotor2{4};

frc::PWMTalonSRX m_grabber{5};

frc::DifferentialDrive m_robotDriveFront{m_left, m_right};
frc::DifferentialDrive m_robotDriveRear{m_left2, m_right2};

frc::XboxController m_controller{0};
frc::Timer m_timer;

class Robot : public frc::TimedRobot {
 public:
 frc::DigitalInput* lms_grabberInward;
 frc::DigitalInput* lms_grabberOutward;

  Robot() {
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDriveFront.SetExpiration(100_ms);
    m_robotDriveRear.SetExpiration(100_ms);
    m_timer.Start();
  }

  void RobotInit() override {
    // Instantiate the DIO objects for port 0 and port 1
    lms_grabberInward = new frc::DigitalInput(0);
    lms_grabberOutward = new frc::DigitalInput(1);
  }

  void AutonomousInit() override {
  }

  void AutonomousPeriodic() override {
    
  }

  void TeleopInit() override {
  }

  void TeleopPeriodic() override {
    double speed = m_controller.GetRawAxis(5);
    double turn = m_controller.GetRawAxis(2);
    m_robotDriveFront.ArcadeDrive(speed, turn);
    m_robotDriveRear.ArcadeDrive(speed, turn);

    bool lms_gInward_state = lms_grabberInward->Get();
    bool lms_gOutward_state = lms_grabberInward->Get();
    if(m_controller.GetRawButton(5) && lms_gInward_state == false){
      m_grabber.Set(0.1);
    }else if(m_controller.GetRawButton(6) && lms_gOutward_state == false){
      m_grabber.Set(-0.1);
    }else{
      m_grabber.Set(0);
    }
  }  
  void TestInit() override {}

  void TestPeriodic() override {}
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif