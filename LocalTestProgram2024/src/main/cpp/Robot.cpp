// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/GenericHID.h>
#include "frc/XboxController.h"
#include "frc/motorcontrol/Spark.h"
#include <ctre/phoenix6/TalonFX.hpp>


using namespace frc;
using namespace rev;

frc::XboxController *controller;
ctre::phoenix6::hardware::TalonFX *intake;
ctre::phoenix6::hardware::TalonFX *shooterLeft;
ctre::phoenix6::hardware::TalonFX *shooterRight;
Spark *chassisBlinkin;
ctre::phoenix6::controls::VelocityVoltage ctreVelocity{0_tps};
units::angular_velocity::turns_per_second_t velTarget = 0.0_tps;



double testingBlinkinVoltage = -.99;
double shooterMultiplier = .5;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Select RPM", 0.0);
  controller = new frc::XboxController(0);
  intake = new ctre::phoenix6::hardware::TalonFX(8);
  shooterLeft = new ctre::phoenix6::hardware::TalonFX(9);
  shooterRight = new ctre::phoenix6::hardware::TalonFX(10);
  shooterRight->SetInverted(true);
  intake->SetInverted(true);

  chassisBlinkin = new Spark(5);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  //Shooter activated by Left/Right Trigger Axises
  intake->Set(controller->GetRightTriggerAxis() - controller->GetLeftTriggerAxis());
  //Intake activated by Right Y Axis
  // if (abs(controller->GetRightY()) > .07) {
  //   shooterLeft->Set(controller->GetRightY()*shooterMultiplier);
  //   shooterRight->Set(controller->GetRightY()*shooterMultiplier);
  // } else {
  //   shooterLeft->Set(.0);
  //   shooterRight->Set(.0);
  // }
  if(controller->GetStartButton()) {
    velTarget = units::angular_velocity::turns_per_second_t{frc::SmartDashboard::GetNumber("Select RPM", 0.0) / 60.0};
    shooterLeft->SetControl(ctreVelocity
      .WithVelocity(velTarget));
    shooterRight->SetControl(ctreVelocity
      .WithVelocity(velTarget));
  } else {
      shooterLeft->Set(0.0);
      shooterRight->Set(0.0);
  }
  frc::SmartDashboard::PutNumber("Actual RPM", shooterLeft->GetVelocity().GetValueAsDouble() * 60.0);
  chassisBlinkin->Set(testingBlinkinVoltage);

  if (controller->GetYButtonReleased()) {
    shooterMultiplier += .05;
  }
  if (controller->GetXButtonReleased()) {
    shooterMultiplier -= .05;
  }


  if (testingBlinkinVoltage > .99) { testingBlinkinVoltage = -.99; } else if (testingBlinkinVoltage < -.99) { testingBlinkinVoltage = .99; }
  if (controller->GetAButtonReleased()) {
    testingBlinkinVoltage += .02;
  }
  if (controller->GetBButtonReleased()) {
    testingBlinkinVoltage -= .02;
  }

  frc::SmartDashboard::PutNumber("Shooter Power", shooterMultiplier);
  frc::SmartDashboard::PutNumber("Blinkin V's", testingBlinkinVoltage);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
