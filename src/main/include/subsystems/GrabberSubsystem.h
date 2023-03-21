// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Config.h"

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
  #if defined(Claw)
    void Open();
    void Close();
    void Toggle( void );
    void IntakeTest();
    const double kRollerGripPercent = 0.15;
  #else
    void Spin( double speed );
    units::ampere_t GetCurrent();
    void GrabberTest();
  #endif
 private:
  double m_spin_speed = 0.0;
  #if defined(Claw)
    ctre::phoenix::motorcontrol::can::TalonFX m_roller{ 14 };
    frc::DoubleSolenoid m_grab{ 9, frc::PneumaticsModuleType::CTREPCM, deviceIDs::kGrabberSolenoidForwardChannel, deviceIDs::kGrabberSolenoidReverseChannel };
  #else
    rev::CANSparkMax m_intake{14, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  #endif
};
