#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/SerialPort.h>
#include <stdio.h>
#include <iostream>
#include <units/time.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Watchdog.h>

frc::PWMTalonSRX m_left{0};
frc::PWMTalonSRX m_right{1};
frc::PWMTalonSRX m_left2{2};
frc::PWMTalonSRX m_right2{3};
frc::DifferentialDrive m_robotDriveFront{m_left, m_right};
frc::DifferentialDrive m_robotDriveRear{m_left2, m_right2};

frc::XboxController m_controller{0};
frc::Timer m_timer;
frc::Timer timer;



class Robot : public frc::TimedRobot {
 public:
  Robot() {
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDriveFront.SetExpiration(100_ms);
    m_robotDriveRear.SetExpiration(100_ms);
    m_timer.Start();
  }
  void RobotInit() {
    serialPort = new frc::SerialPort(9600, frc::SerialPort::kMXP);
  }

  void RobotPeriodic() {
   
    
    if (serialPort.GetBytesReceived() > 0) {
    int data = serialPort->ReadInt(); // read up to 100 bytes from the serial port
    frc::DriverStation::ReportError(data); // print the received data to the console
  }
  }

  void AutonomousInit() override {
  }

  void AutonomousPeriodic() override {
    
  }

  void TeleopInit() override {
  
    frc::Watchdog watchdog{units::second_t{0.1}, []{} };
    // ... other code ...
    watchdog.Disable();  // disable the watchdog
    timer.Start();

  }

  void TeleopPeriodic() override {
    
    if(timer.Get().to<double>() > 5.0){

    }
    if(arduinoSerial.GetBytesReceived() > 0)
    {
      //std::cout << "test";
      char data[256];
      int count = arduinoSerial.Read(data, sizeof(data));
      int value = atoi(data);
      std::string message = std::to_string(value);
      frc::SmartDashboard::PutString("My Message", message);
    }

    double speed = m_controller.GetRawAxis(5);
    double turn = m_controller.GetRawAxis(2);
    m_robotDriveFront.ArcadeDrive(speed, turn);
    m_robotDriveRear.ArcadeDrive(speed, turn);
  }

  void TestInit() override {}

  void TestPeriodic() override {}

  void MyCallback() {
    frc::SmartDashboard::PutString("My Message", "Timer Expired Fuck You!");
}
private:
frc::SerialPort* serialPort;
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
