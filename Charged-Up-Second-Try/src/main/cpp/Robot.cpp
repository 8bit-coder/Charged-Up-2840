#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/SerialPort.h>
#include <ctre/Phoenix.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>

//yike
ctre::phoenix::motorcontrol::can::TalonFX joint1Motor(0);
ctre::phoenix::motorcontrol::can::TalonFX joint2Motor(1);

auto sensorCollectionMotor1 = joint1Motor.GetSensorCollection();
auto sensorCollectionMotor2 = joint2Motor.GetSensorCollection();

double currJoint1Pos = sensorCollectionMotor1.GetIntegratedSensorAbsolutePosition();
double currJoint2Pos = sensorCollectionMotor2.GetIntegratedSensorAbsolutePosition();

double endPoint[2]; //distance from origin
double origin[2]; //always 0

double a1; //length of first arm segment
double a2; //length of second arm segment

double q1;
double q2;

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

    a1 = 10;
    a2 = 10;

    origin[0] = 0;
    origin[1] = 0;
  }
  void RobotPeriodic() override
  {
    constexpr int kNumBytesToRead = 3; //num of digits + 2

    currJoint1Pos = map(currJoint1Pos, 0, 2048, 0, 360);
    currJoint2Pos = map(currJoint2Pos, 0, 2047, 0, 359);

    endPoint[0] = 1;
    endPoint[1] = 1;

    double sqrDist = pow(endPoint[0],2) + pow(endPoint[1],2);

    if(sqrt(sqrDist) > a1 + a2)
    {
      endPoint[0] = endPoint[0] * ((a1 + a2) / sqrt(sqrDist));
      endPoint[1] = endPoint[1] * ((a1 + a2) / sqrt(sqrDist));
    }

    q2 = acos(((endPoint[0] - origin[0])*(endPoint[0] - origin[0]) + (endPoint[1] - origin[1])*(endPoint[1] - origin[1]) + (-a1*a1) + (-a2*a2))/(2*a1*a2));
    q1 = atan2((endPoint[1] - origin[1]),(endPoint[0] - origin[0])) - atan2((a2*sin(q2)),(a1 + a2*cos(q2))) ;
 
    int targetEncoderValue1 = static_cast<int>(std::round(map(q1, 0, 360, 0, 2048)));
    int targetEncoderValue2 = static_cast<int>(std::round(map(q2, 0, 360, 0, 2048)));

    joint1Motor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, targetEncoderValue1);
    joint2Motor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, targetEncoderValue2);

    if(arduinoSerial.GetBytesReceived() > 0)
    {
      char buffer[kNumBytesToRead];
      int num_bytes_read = arduinoSerial.Read(buffer, kNumBytesToRead);
      //frc::SmartDashboard::PutString("My", "hee");
      /*for(int i = 0; i < sizeof(buffer) / sizeof(buffer[0]); i++)
      {
        frc::SmartDashboard::PutString(std::to_string(i), buffer);
      }*/
      std::string str(reinterpret_cast<char*>(buffer), num_bytes_read);
      frc::SmartDashboard::PutString("MyIntegerValue", str);
    }
    else
    {
      frc::SmartDashboard::PutString("Message", "no bytes recieved");
    }
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

  double map(double value, double start1, double stop1, double start2, double stop2)
 {
    return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
  }
};



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif