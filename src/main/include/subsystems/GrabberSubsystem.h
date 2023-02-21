// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/DoubleSolenoid.h>

class GrabberSubsystem : public frc2::SubsystemBase {
 public:
  GrabberSubsystem();

  void Periodic() override;

  void Open();

  void Close();

  void Spin( double speed );

 private:
  
  ctre::phoenix::motorcontrol::can::TalonFX m_roller{ 11 };
  frc::DoubleSolenoid m_grab{ frc::PneumaticsModuleType::REVPH, 1, 2 };
};
