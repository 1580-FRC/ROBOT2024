// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <chrono>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <networktables/NetworkTable.h>


void Robot::SetArmPow(double p)
{
  frc::SmartDashboard::PutNumber("Arm Power", p);
  p = -p;

  armVictor1.Set(p);
  armVictor2.Set(p);
}

void Robot::RobotInit()
{
  // flexShoot.controller.SetSmartCurrentLimit(35);
  // armSparky2.SetInverted(true);
  // armSparky2.SetInverted(true);
  std::cout << "INIT!" << std::endl;
  this->parash = MaslulParash::ArmUp;

  // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
  rotationPid.SetTolerance(5, 10);
  rotationPid.EnableContinuousInput(0, 360);
  isRotating = false;
  autoArm = false;
  intakePositionOn = false;
  simaFlag = false;

  armPid.SetTolerance(5);
  // rotatingState = RotatingState::Idle;
  // armState = ArmState::Idle;
  // encoder.SetPositionConversionFactor(360);  // degrees, absolute mode reports between 0 and 1, multiply by 360 and we have detected degrees
  encoder.SetPositionOffset(130.0 / 360.0);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}


double Robot::ArmAngle()
{
  const double offset = 315.79 + 7 + 185 - 63; //change offset as needed
  double sima = this->encoder.GetAbsolutePosition() * 360 - offset;
  // return this->encoder.GetAbsolutePosition() * 360.0 - offset;
  if (sima < -1)
  {
    return sima + 360;
  }
  else if (round(sima) == -1)
  {
    return 0;
  }
  return sima;
}

double ConstantArmPowerAngle(double a)
{

  const double cosFactor = cos(a / 180.0 * M_PI);
  if (a > 80)
  {
    return 0.3 * cosFactor;
  }

  return 0.2977 * cosFactor;
}

double Robot::CalculatePower(double angle)
{
  // Assuming angle is in the range [0, 90]
  // if (angle >= 0 && angle <= 90) {
  // double power = -0.016 * angle + 1;
  double current = this->ArmAngle();
  double power = ConstantArmPowerAngle(current);
  bool isAuto = this->timer.Get().value() <= 15;

  double angleDiffRatio = ((this->targetAngle - current) / 90);
  if (current < this->targetAngle && !simaFlag)
  {
    // UP
    // std::cout << "PLUS" << power << std::endl;
    power += 0.01 + 0.15 * angleDiffRatio;
    if (isAuto) {
      std::cout << "AUTO !!" << std::endl;
      power += 0.105;
    }
  }
  else if(current >= 100 && current < 115 && !simaFlag)
  {
    power -= 0.23;
  }
  else if(current >= 115 && !simaFlag)
  {
    power = 0;
  }
  else if (this->ArmAngle() < 85 && !simaFlag)
  {
    // 0.155
    power -= 0.05 - 0.05 * angleDiffRatio;
    // power = std::max(power, ConstantArmPowerAngle(current));
    // power = std::max(power, ConstantArmP)
    // power -= 0.055 - 0.9 * angleDiffRatio;
  }
  else if (this->ArmAngle() > 90 && simaFlag)
  {
    power = -0.53 * (sin(this->ArmAngle() / 180.0 * M_PI));
  }

  if (this->ArmAngle() < 20 && this->intakePositionOn)
  {
    power = 0;
    autoArm = false;
  }
  // else {
  //   power -= 0.115;
  // }

  // std::cout << "POWER" << power << std::endl;

  // power = std::max(-power, ConstantArmPowerAngle(angle));
  return power;
  // } else {
  //     // Handle invalid input (outside the range [0, 90])
  //     std::cerr << "Error: Invalid angle input. Angle must be between 0 and 90 degrees." << std::endl;
  //     // You might want to return a default value or handle the error in a way that suits your needs.
  //     return 0.0;
  // }
}
/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  this->flagToPass = false;
  this->didShot = false;

  // std::cout << EstimateDistance() << std::endl;

  // std::cout << "ENCODER " << this->ArmAngle() << std::endl;
  //  std::cout << "ENCODER " << -1 * calculatePower(round(this->ArmAngle())) << std::endl;

  double yaw = gyro.GetYaw();
  frc::SmartDashboard::PutNumber("TV", LimelightHelpers::getTV());
  frc::SmartDashboard::PutNumber("Distancee", EstimateDistance());
  frc::SmartDashboard::PutNumber("Proximity", this->proximity.GetProximity());
  frc::SmartDashboard::PutNumber("Arm Angle", this->ArmAngle());
  frc::SmartDashboard::PutNumber("Drive Sens", this->GetDrivingSense());
  frc::SmartDashboard::PutNumber("Yaw", yaw);
  frc::SmartDashboard::PutNumber("Yaw Graph", yaw);
  frc::SmartDashboard::PutBoolean("SIMA", simaFlag);
  frc::SmartDashboard::PutBoolean("SHOOT POWER", xbox.GetRawAxis(SHOOT_AXIS));
  frc::SmartDashboard::PutBoolean("NEG SHOOT POWER", xbox.GetRawAxis(REV_SHOOT_AXIS));

  if (autoArm)
  {
      this->SetArmPow(this->CalculatePower(this->targetAngle));
  }

   if (isRotating)
  {
    double power = rotationPid.Calculate(yaw);

    Move(power, -power, 0.075);
  }

#ifdef DIST_SENSOR
  distCm = dist_sensor.GetValue() * 5/frc::RobotController::GetVoltage5V() * 0.125;
#endif
  frc::SmartDashboard::PutNumber("DIST sensor", distCm);
}

double Robot::GetDrivingSense()
{

  return (-leftStick.GetZ() + 1) / 2;
}

double Robot::GetRightSense()
{

  return (-rightStick.GetZ() + 1) / 2;
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
const double MAX_SHOOT = 0.65;
void Robot::AutonomousInit()
{
  this->runner = new ActionRunner(GetPlanCenter(*this));

  #ifdef DIST_SENSOR
  ultrasonic_trigger_pin.Set(1);
  #endif
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  // warmupStart = std::chrono::steady_clock::now();
  this->parash = MaslulParash::ArmUp;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

bool targetDetected()
{
  const double detected = 1;

  return (LimelightHelpers::getTV() == detected);
}
void Robot::advanceAuto()
{
  this->parash = static_cast<MaslulParash>(static_cast<int>(this->parash) + 1);
}

void Robot::AdvanceIfArm(double diff)
{
  if (std::abs(this->targetAngle - this->ArmAngle()) < diff)
  {
    this->advanceAuto();
  }
}

void Robot::TimerMs(int t)
{
  timerMs = t;
  if (timerEnabled)
  {
    auto now = std::chrono::steady_clock::now();

    double diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - timerStart).count();
    if (diff >= timerMs)
    {
      timerEnabled = false;
      advanceAuto();
    }
  }
  else
  {
    timerEnabled = true;
    timerStart = std::chrono::steady_clock::now();
  }
}

void Robot::AutonomousPeriodic()
{
  if (this->runner != nullptr){
    this->runner->Update((Robot&)*this);
  }
  // if (m_autoSelected == kAutoNameCustom)
  // {
  //   // Custom Auto goes here
  // }/
  // else
  // {
  bool isAuto = this->timer.Get().value() <= 15;
  if (!isAuto) {
    this->flexShoot.Set(0);
    return;
  }


  const int TIME = 3000;
  const int TIME2 = 1300;
  
  frc::SmartDashboard::PutNumber("State", (int)this->parash);
  // switch (parash)
  // {
  // case MaslulParash::ArmUp:
  // {
  //   // this->SetArmPow(0.243);
  //   // this->TimerMs(675);
  //   this->SetArming(85);
  //   this->AdvanceIfArm(10);
  //   break;
  // }
  // case MaslulParash::ArmDown:
  //   this->Shoot();
  //   this->SetArming(RESTING_ARM_ANGLE);
  //   this->AdvanceIfArm(5);
  //   break;
  // // case MaslulParash::Warmup:
  // //   this->TimerMs(500);0
  // //   break;
  // case MaslulParash::Wait:
  //   this->TimerMs(1500);
  // break;
  // case MaslulParash::Shoot:
  //   // this->Shoot();
  //   this->IntakeToShooter();
  //   // this->advanceAuto();
  //   break;
  // case MaslulParash::EnablePickup:
  //   this->IntakeOn(true);
  //   this->flexShoot.Set(0);
  //   this->advanceAuto();
  //   break;
  // case MaslulParash::Forward:
  //   this->MoveUntilDist(TIME, true);
  //   // this->MoveUntilDist(121);
  //   break;
  // case MaslulParash::WaitStop: 
  //   this->Shoot();
  //   this->TimerMs(1500);
  // break;
  // case MaslulParash::Backward:
  //   this->MoveUntilDist(TIME);
  //   break;
  // case MaslulParash::Shoot2:
  //   // this->Shoot();
  //   this->IntakeToShooter();

  //   break;

  // case MaslulParash::Turn:
  // {
  //   this->flexShoot.Set(0);
  //   //57, 45.1 robot -> note
  //   // const int offset = 3;
  //   // // const double wantedAngle = 45;
  //   // const double wantedAngle = 51.647;
  //   // // const double wantedAngle = 360 - 51.647;
  //   // this->SetRotating(wantedAngle);

  //   // if (gyro.GetAngle() > wantedAngle - offset && gyro.GetAngle() < wantedAngle + offset)
  //   // {
  //   //   this->advanceAuto();
  //   // }
  //   // turn nahoi
  //   break;
  // }
  // // case MaslulParash::Forward2:
  // //   // hyp   70.5
  // //   this->MoveUntilDist(TIME2);
  // //   // should pickup
  // //   break;
  // // case MaslulParash::Backward2:
  // //   this->MoveUntilDist(TIME2, true);
  // // break;
  // // case MaslulParash::Shoot3:
  // //   this->IntakeToShooter();
  // //   break;
  // // case MaslulParash::Forward3:
  // //   this->MoveUntilDist(10);
  // //   break;
  // // case MaslulParash::Shoot4:
  // //   this->Shoot();
  // //   break;
  // }

    const double proximityLimit = 300;
    // if loaded we are pushing to shooter
    if (proximity.GetProximity() > proximityLimit && !loaded)
    {
      this->IntakeOff();
      loaded = true;
      if (this->parash == MaslulParash::Forward) {
        this->advanceAuto();
      }
      // intakeOn = false;
    }


    // }
}


void Robot::IntakeOn(bool half)
{
  intakeOn = true;

if (half) {
  flexIntake.Set(PICKUP_POWER / 2);

}else { 
  flexIntake.Set(PICKUP_POWER);

}
}

void Robot::IntakeToShooter()
{
  IntakeOn();
}

void Robot::IntakeOff()
{
  intakeOn = false;
  flexIntake.Set(0);
}
bool Robot::MoveUntilDist(double distMax, bool opposite)
{
  // x0 + v0t + 0.5 * a * t^2
  // vMax * t
  this->TimerMs((int)distMax);

  if (!timerEnabled)
  {
    this->Move(0, 0, 1);
    // this->advanceAuto();
    return true;
  }

  double MUL = 0.3;
  if (opposite)
  {
    MUL *= -1;
  }
  this->Move(1, 1, MUL);

  return false;

  // if (targetDetected())
  // {
  //   double distanceHyp = EstimateDistance();
  //   double tx = LimelightHelpers::getTX();

  //   const double stageDisVeritcal = 5;

  //   double curX = sqrt(distanceHyp * distanceHyp - stageDisVeritcal * stageDisVeritcal);
  //   // calc angle

  //   if (curX > distMax)
  //   {
  //     this->advanceAuto();
  //     this->Move(0, 0, 1);
  //     return true;
  //   }
  // }
}

void Robot::Shoot()
{
  flexShoot.Set(-MAX_SHOOT);
}

double Robot::ConstantArmPower()
{
  return ConstantArmPowerAngle(this->ArmAngle());
}

void Robot::TeleopInit() {
  this->flexShoot.Set(0);
  this->flexIntake.Set(0);
}
// std::

void Robot::TeleopPeriodic()
{
  double sens = GetDrivingSense();
  // double yaw = gyro.GetYaw();
  double power = rightStick.GetY();
  Move(rightStick.GetY(), leftStick.GetY(), sens);

  double armPower = -1;

  bool activateTest = false;
  bool asherFlag = false;
  if (leftStick.GetRawButtonPressed(1))
  {
    activateTest = true;
    // this->SetArmPow(-1* calculatePower(round(this->ArmAngle())));
    // std::cout << armPower * sens << std::endl;
    //  armPid.SetSetpoint(this->ArmAngle());
  }
  // if(this->ArmAngle() > 85)
  // {
  //   activateTest = false;
  //   asherFlag = true;
  //   std::cout << "SIMA " << std::endl;
  // }
  // if(activateTest)
  // {
  //   this->SetArmPow(-1 * calculatePower(round(this->ArmAngle())));
  // }
  // if(asherFlag)
  // {
  //   this->SetArmPow(0);
  // }

  if (xbox.GetRawButtonPressed(2))
  {
    simaFlag = false;
    autoArm = false;
    this->SetArmPow(0.05);
    std::cout << "bimba" << std::endl;
  }
  if (xbox.GetRawButtonReleased(2) || this->ArmAngle() < 10)
  {
    simaFlag = false;
    this->SetArmPow(0);
  }

  else if (rightStick.GetRawButtonPressed(2))
  {
    this->SetArmPow(0.25);
    // armPid.SetSetpoint(this->ArmAngle());
  }
  else if (rightStick.GetRawButtonReleased(2))
  {
    this->SetArmPow(0);
    // armPid.SetSetpoint(this->ArmAngle());
  }
  else if (leftStick.GetRawButtonPressed(3))
  {
    this->SetArmPow(0);
  }

  if (xbox.GetRawButtonPressed(4))
  {
    simaFlag = false;
    this->SetArming(90);
  }
  else if (xbox.GetRawButtonPressed(1))
  {
    simaFlag = false;
    this->SetArming(RESTING_ARM_ANGLE);
  }
  else if (rightStick.GetRawButtonPressed(8))
  {
    autoArm = false;
    this->SetArmPow(ConstantArmPowerAngle(RESTING_ARM_ANGLE));
  }

  if (leftStick.GetRawButtonPressed(6))
  {
    this->SetRotating(0);
  }
  else if (leftStick.GetRawButtonPressed(7))
  {
    this->SetRotating(90);
  }
  else if (leftStick.GetRawButtonPressed(10))
  {
    this->SetRotating(180);
  }
  else if (leftStick.GetRawButtonPressed(11))
  {
    this->SetRotating(270);
  }

  if (power > 0.05)
  {
    // stop rotating if driver takes control
    isRotating = false;
  }


  // this->SetArmPow(this->GetRightSense());
  // hold arm
  // this->SetArmPow(this->ConstantArmPower());
  // + this->ConstantArmPower()

  // }

  // double pickupPower = xbox.GetRawAxis(2);
  // if (pickupPower){
  //   pickupPower = std::min(pickupPower, 0.3);
  //   flexPickup.Set(pickupPower);
  // }else {
  //   flexPickup.Set(0.0);
  // }
  const int PICKUP_BUTTON_OUT = 6;
  double current = this->ArmAngle();
  const int PICKUP_BUTTON_IN = 5;

  /*


  // toggle on
  if(xbox.GetRawButtonPressed(PICKUP_BUTTON_IN))
  {
    // intakePositionOn = true;
    intakeOn = !intakeOn;
    // flexIntake.Set(PICKUP_POWER);

    if (intakeOn) {
      flexIntake.Set(PICKUP_POWER);
      IntakeOn();
    }else {
      IntakeOff();
    }
  }*/
  if (xbox.GetRawButtonPressed(PICKUP_BUTTON_IN))
  {
    this->IntakeOn();
  }
  else if (xbox.GetRawButtonReleased(PICKUP_BUTTON_IN))
  {
    this->IntakeOff();
  }

  // elif released for normal but i don't want noremal i want proximity

  if (xbox.GetRawButtonPressed(PICKUP_BUTTON_OUT))
  {
    flexIntake.Set(-PICKUP_POWER);
  }
  else if (xbox.GetRawButtonReleased(PICKUP_BUTTON_OUT))
  {
    flexIntake.Set(0);
  }

  double shootPower = xbox.GetRawAxis(SHOOT_AXIS);
  double negShootPower = xbox.GetRawAxis(REV_SHOOT_AXIS);
  if (shootPower > 0.0)
  {
    shootPower = std::min(shootPower, MAX_SHOOT);
    flexShoot.Set(-shootPower);
  }else if (negShootPower > 0.0) {
    negShootPower = std::min(negShootPower, MAX_SHOOT);
    flexShoot.Set(negShootPower);
  }
  else
  {
    flexShoot.Set(0.0);

    // flexShoot.controller.;
  }

  double offset = 3.5;

  if (xbox.GetPOV() == 0)
  {
    this->SetArming(this->targetAngle + offset);
  }
  else if (xbox.GetPOV() == 270)
  {
    offset -= 3;
  }
  else if (xbox.GetPOV() == 180)
  {
    simaFlag = true;
    this->SetArming(this->targetAngle - offset);
  }
  else if (xbox.GetPOV() == 90)
  {
    offset += 3;
  }

  // double pickupPower = xbox.GetRawAxis(2);
  // if (pickupPower){
  //   pickupPower = std::max(pickupPower, 0.85);
  //   flexPickup.Set(pickupPower);
  // }


}

void Robot::SetRotating(double d)
{
  // rotationPid.Reset();
  this->isRotating = true;
  rotationPid.SetSetpoint(d);
}

void Robot::SetArming(double d)
{
  // rotationPid.Reset();
  this->autoArm = true;
  targetAngle = d;
  frc::SmartDashboard::PutNumber("Target Angle", d);
  // armPid.SetSetpoint(d);

  if (targetAngle != RESTING_ARM_ANGLE)
  {
    intakePositionOn = false;
  }
  else {
    intakePositionOn = true;
  }
}

double Robot::EstimateDistance()
{

  // if (LimelightHelpers::getTA() > 8)
  // {
  //   // std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  //   // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  //   double targetOffsetAngle_Vertical = LimelightHelpers::getTY();

  //   // how many degrees back is your limelight rotated from perfectly vertical?
  //   double limelightMountAngleDegrees = 18.0;

  //   // distance from the center of the Limelight lens to the floor
  //   double limelightLensHeightInches = 20.0;

  //   // distance from the target to the floor
  //   double goalHeightInches = 60.0;

  //   double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  //   double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //   // calculate distance
  //   double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);
  //   return distanceFromLimelightToGoalInches * 2.54;
  // }
  return -1.0;
}
void Robot::Move(double right, double left, double sens)
{

  frc::SmartDashboard::PutNumber("Move Power", right * sens);

  sparkyDrive1.Set(-right * sens);
  sparkyDrive2.Set(-right * sens);
  sparkyDrive3.Set(left * sens);
  sparkyDrive4.Set(left * sens);
}
void Robot::DisabledInit() {}

void Robot::
    DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic()
{
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
