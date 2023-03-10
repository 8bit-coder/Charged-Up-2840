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
ctre::phoenix::motorcontrol::can::TalonFX testMotor(0);

auto sensorCollectionMotor1 = testMotor.GetSensorCollection();

double position = sensorCollectionMotor1.GetIntegratedSensorAbsolutePosition();

frc::PWMTalonSRX m_left{0};
frc::PWMTalonSRX m_right{1};
frc::PWMTalonSRX m_left2{2};
frc::PWMTalonSRX m_right2{3};
frc::PWMTalonSRX m_grabber{4};
frc::DifferentialDrive m_robotDriveFront{m_left, m_right};
frc::DifferentialDrive m_robotDriveRear{m_left2, m_right2};

frc::SerialPort arduinoSerial {9600, frc::SerialPort::Port::kMXP, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One};

frc::XboxController m_controller{0};
frc::Timer m_timer;

class Robot : public frc::TimedRobot {
 public:
 frc::DigitalInput* lms_inward;
 frc::DigitalInput* lms_outward;

  Robot() {
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDriveFront.SetExpiration(100_ms);
    m_robotDriveRear.SetExpiration(100_ms);
    m_timer.Start();
  }

  void RobotInit() override {
    // Instantiate the DIO objects for port 0 and port 1
    lms_inward = new frc::DigitalInput(0);
    lms_outward = new frc::DigitalInput(1);
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

    bool lms_inward_state = lms_inward->Get();
    bool lms_outward_state = lms_outward->Get();
    if(m_controller.GetRawButton(5) && lms_inward_state == false){
      m_grabber.Set(0.1);
    }else if(m_controller.GetRawButton(6) && lms_outward_state == false){
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
