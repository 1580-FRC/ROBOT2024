// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "LimelightHelpers.h"
#include <string>
#include <rev/CANSparkFlex.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>



#include "frc/DigitalInput.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <string>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <ctime>
#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/Victor.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <rev/CANSparkMax.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/filter/LinearFilter.h>
#include <networktables/DoubleTopic.h>
#include <frc/DigitalInput.h>
#include <rev/ColorSensorV3.h>
#include <frc/motorcontrol/VictorSP.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

template <class T, double MAX, class... Args>
class EncoderLimit {
  public: 
  T controller;
  EncoderLimit(Args... args) : controller(args...){
    
  }
  
  void Set(double d) {
    d = std::clamp(d, -MAX, MAX);
    this->controller.Set(d);
  }
};

enum class MaslulParash {
  ArmUp,
  ArmDown,
  Wait,
  // Warmup,
  Shoot,
  EnablePickup,
  Forward,
  WaitStop,
  Backward,
  WarmUp,
  Shoot2,
  Turn,
  Forward2,
  Backward2,
  // turn to speaker
  Shoot3,
  // turn bact to note
  Forward3,
  // turn to speaker
  Shoot4,
};

class Robot : public frc::TimedRobot
{
public:
  double GetDrivingSense();
  double GetRightSense();
  void Shoot();
  bool MoveUntilDist(double dist, bool opposite = false);
  void AdvanceIfArm(double diff);
  const int SHOOT_AXIS = 3;
  const int REV_SHOOT_AXIS = 3;

  void TimerMs(int t );
  double ArmAngle();
  double ConstantArmPower();
  void SetArmPow(double p);
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
private:
  // ArmState armState = ArmState::Idle;
  // RotatingState rotatingState = RotatingState::Idle;
  bool isRotating = false;
  bool autoArm = false;
  bool intakePositionOn = false;
  bool intakeOn = false;
  const double PICKUP_POWER = 0.2;
  const int INTAKE_TO_SHOOTER_TIME = 1000;
  const int TIME_TO_2 = 500;

  frc::Timer timer;
  bool timerEnabled = false;
  int timerMs = 0;
  std::chrono::steady_clock::time_point timerStart;

  double targetAngle = 0.0;
  MaslulParash parash = (MaslulParash)0; 
  frc::PIDController rotationPid{0.275, 0.675, 0.05};
  void advanceAuto();
  void IntakeOn(bool half = false);
  void IntakeToShooter();
  void IntakeOff();
  
  // frc::PIDController armPid{2, 0.8, 0.05};
  frc::ProfiledPIDController<units::angle::radian> armPid{
  0.05, 0.7, 0.01,
  // max ~30 deg per sec
  frc::TrapezoidProfile<units::angle::radian>::Constraints{units::angular_velocity::radians_per_second_t{0.5}, units::angular_acceleration::radians_per_second_squared_t{1}}};
  
  frc::ArmFeedforward armFeed{units::volt_t{1.5}, units::volt_t{ 2.4}, units::unit_t<frc::ArmFeedforward::kv_unit>{1.89}, units::unit_t<frc::ArmFeedforward::ka_unit>{0.15}};
  
  rev::CANSparkMax sparkyDrive1{1, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkMax sparkyDrive2{2, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkMax sparkyDrive3{3, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkMax sparkyDrive4{4, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkFlex flexShoot{9, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex flexIntake{10, rev::CANSparkLowLevel::MotorType::kBrushless};
  EncoderLimit<frc::VictorSP, 0.85, int> armVictor1{1};
  EncoderLimit<frc::VictorSP, 0.85, int> armVictor2{0};

  frc::Joystick leftStick{0};
  frc::Joystick rightStick{1};
  frc::Joystick xbox{2};

  double last_arm_pos = 0.0;
  bool simaFlag = false;
  AHRS gyro{frc::SerialPort::Port::kUSB};
  // frc::SimpleMotorFeedForward shooterFeedForward{};
  //rev::ColorSensorV3 colorSensor{ frc::I2C::Port::kOnboard};


  // rev::SparkAbsoluteEncoder encoder = armSparky.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
  frc::DutyCycleEncoder armEncoder{3};

  // rev::SparkMaxRelativeEncoder encoder = armSparky.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kQuadrature);

  void Move(double right, double left, double sens);
  double CalculatePower(double angle);
  void SetRotating(double angle);
  void SetArming(double angle);
  double EstimateDistance();

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 proximity{i2cPort};
  //frc::DigitalInput actionSwitch{1};
  bool flagToPass;
  bool didShot;
};
