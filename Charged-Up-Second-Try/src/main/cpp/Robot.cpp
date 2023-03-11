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
#include <frc/motorcontrol/MotorControllerGroup.h>
//yike
frc::PWMTalonSRX m_left{0};
frc::PWMTalonSRX m_right{1};
frc::PWMTalonSRX m_left2{2};
frc::PWMTalonSRX m_right2{3};

frc::MotorControllerGroup m_drivetrainLeft{m_left, m_left2};
frc::MotorControllerGroup m_drivetrainRight{m_right, m_right2};

frc::DifferentialDrive m_robotDrive{m_drivetrainLeft, m_drivetrainRight};

frc::PWMTalonSRX m_armTiltMotor1{4};
frc::PWMTalonSRX m_armTiltMotor2{5};

frc::MotorControllerGroup m_armTiltMotors{m_armTiltMotor1, m_armTiltMotor2};

frc::PWMTalonSRX m_grabberTiltMotor{6};

frc::PWMTalonSRX m_grabber{6};

frc::XboxController m_controller{0};
frc::Timer m_timer;

class Robot : public frc::TimedRobot {
 public:
  frc::DigitalInput* lms_grabberInward = new frc::DigitalInput(0);
  frc::DigitalInput* lms_grabberOutward = new frc::DigitalInput(1);

  frc::DigitalInput* lms_armUpper = new frc::DigitalInput(2);
  frc::DigitalInput* lms_armLower = new frc::DigitalInput(3);

  frc::DigitalInput* lms_grabberTiltUpper = new frc::DigitalInput(4);
  frc::DigitalInput* lms_grabberTiltLower = new frc::DigitalInput(5);

  double turn = 0.0;
  double speed = 0.0;

  Robot() {
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }

  void RobotInit() override {}

  void AutonomousInit() override {}

  void AutonomousPeriodic() override {

  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    speed = m_controller.GetRawAxis(1);
    turn = m_controller.GetRawAxis(0);
    m_robotDrive.ArcadeDrive(speed, turn);

    if(m_controller.GetRawButton(5) && lms_grabberInward->Get() == false){
      m_grabber.Set(0.1);
    }else if(m_controller.GetRawButton(6) && lms_grabberOutward->Get() == false){
      m_grabber.Set(-0.1);
    }else{
      m_grabber.Set(0);
    }

    if(m_controller.GetPOV() == 0 && lms_armUpper->Get() == false){
      m_armTiltMotors.Set(0.3);
    }
    if(m_controller.GetPOV() == 180 && lms_armLower->Get() == false){
      m_armTiltMotors.Set(-0.3);
    }

    if(m_controller.GetRawButton(4) && lms_grabberTiltUpper->Get() == false){
      m_grabberTiltMotor.Set(0.3);
    }
    if(m_controller.GetRawButton(2) && lms_grabberTiltLower->Get() == false){
      m_grabberTiltMotor.Set(-0.3);
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