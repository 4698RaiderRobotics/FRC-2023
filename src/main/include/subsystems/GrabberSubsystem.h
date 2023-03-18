// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/current.h>
#include "Constants.h"

class GrabberSubsystem : public frc2::SubsystemBase {
 public:
  GrabberSubsystem();

  void Periodic() override;

  //void Open();

  //void Close();

  void Spin( double speed );

  units::ampere_t GetCurrent();
  //void Toggle( void );
  //void IntakeTest();
  void GrabberTest();

  const double kRollerGripPercent = 0.15;

 private:
  double m_spin_speed = 0.0;
  rev::CANSparkMax m_intake{14, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  //ctre::phoenix::motorcontrol::can::TalonFX m_roller{ 14 };
  //frc::DoubleSolenoid m_grab{ 9, frc::PneumaticsModuleType::CTREPCM, deviceIDs::kGrabberSolenoidForwardChannel, deviceIDs::kGrabberSolenoidReverseChannel };
};
