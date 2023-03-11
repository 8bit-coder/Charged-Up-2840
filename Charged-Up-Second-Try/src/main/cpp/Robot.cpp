/* 
  General Information:
  m_ means Motor Controller
  mg_ means Motor Controller Group
  dd_ means Differential Drive
  lms_ means Limit Switch
  t_ means Timer
  c_ means Controller
*/

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

frc::PWMTalonSRX m_left{0};
frc::PWMTalonSRX m_right{1};
frc::PWMTalonSRX m_left2{2};
frc::PWMTalonSRX m_right2{3};

frc::MotorControllerGroup mg_drivetrainLeft{m_left, m_left2};
frc::MotorControllerGroup mg_drivetrainRight{m_right, m_right2};

frc::DifferentialDrive dd_robotDrive{mg_drivetrainLeft, mg_drivetrainRight};

frc::PWMTalonSRX m_armTiltMotor1{4};
frc::PWMTalonSRX m_armTiltMotor2{5};

frc::MotorControllerGroup mg_armTiltMotors{m_armTiltMotor1, m_armTiltMotor2};

frc::PWMTalonSRX m_grabberTiltMotor{6};

frc::PWMTalonSRX m_grabber{7};

frc::XboxController c_ps5Controller{0};
frc::Timer t_timer;

class Robot : public frc::TimedRobot {
 public:
  frc::DigitalInput* lms_grabberInward = new frc::DigitalInput(0);
  frc::DigitalInput* lms_grabberOutward = new frc::DigitalInput(1);

  frc::DigitalInput* lms_armTiltUpper = new frc::DigitalInput(2);
  frc::DigitalInput* lms_armTiltLower = new frc::DigitalInput(3);

  frc::DigitalInput* lms_grabberTiltUpper = new frc::DigitalInput(4);
  frc::DigitalInput* lms_grabberTiltLower = new frc::DigitalInput(5);

  double speed = 0.0;
  double turn = 0.0;

  Robot() {
    mg_drivetrainRight.SetInverted(true);
    dd_robotDrive.SetExpiration(100_ms);
    t_timer.Start();
  }

  void RobotInit() override {}

  void AutonomousInit() override {}

  void AutonomousPeriodic() override {
    dd_robotDrive.ArcadeDrive(0.2, 0);
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    speed = c_ps5Controller.GetRawAxis(1);
    turn = c_ps5Controller.GetRawAxis(0);
    dd_robotDrive.ArcadeDrive(speed, turn);

    if(c_ps5Controller.GetRawButton(5) && lms_grabberInward->Get() == false){
      m_grabber.Set(0.1);
    }else if(c_ps5Controller.GetRawButton(6) && lms_grabberOutward->Get() == false){
      m_grabber.Set(-0.1);
    }else{
      m_grabber.Set(0);
    }

    if(c_ps5Controller.GetPOV() == 0 && lms_armTiltUpper->Get() == false){
      mg_armTiltMotors.Set(0.3);
    }
    if(c_ps5Controller.GetPOV() == 180 && lms_armTiltLower->Get() == false){
      mg_armTiltMotors.Set(-0.3);
    }

    if(c_ps5Controller.GetRawButton(4) && lms_grabberTiltUpper->Get() == false){
      m_grabberTiltMotor.Set(0.3);
    }
    if(c_ps5Controller.GetRawButton(2) && lms_grabberTiltLower->Get() == false){
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