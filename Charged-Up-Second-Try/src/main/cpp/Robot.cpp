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
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <stdio.h>
#include <iostream>

using namespace frc;

PWMTalonSRX m_left{0};
PWMTalonSRX m_right{1};
PWMTalonSRX m_left2{2};
PWMTalonSRX m_right2{3};

MotorControllerGroup mg_drivetrainLeft{m_left, m_left2};
MotorControllerGroup mg_drivetrainRight{m_right, m_right2};

DifferentialDrive dd_robotDrive{mg_drivetrainLeft, mg_drivetrainRight};

PWMTalonSRX m_armTiltMotor1{4};
PWMTalonSRX m_armTiltMotor2{5};

MotorControllerGroup mg_armTiltMotors{m_armTiltMotor1, m_armTiltMotor2};

PWMTalonSRX m_grabberTiltMotor{6};

PWMTalonSRX m_grabber{7};

XboxController c_ps5Controller{0};

Timer t_timer;

class Robot : public frc::TimedRobot {
 public:
  DigitalInput* lms_grabberInward = new DigitalInput(0);
  DigitalInput* lms_grabberOutward = new DigitalInput(1);

  DigitalInput* lms_armTiltUpper = new DigitalInput(2);
  DigitalInput* lms_armTiltLower = new DigitalInput(3);

  DigitalInput* lms_grabberTiltUpper = new DigitalInput(4);
  DigitalInput* lms_grabberTiltLower = new DigitalInput(5);

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