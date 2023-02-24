// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>

#include "Constants.h"
#include "AbsoluteEncoder.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;

  void Arm( units::degree_t angle );

  void ArmTestSetup();

  void ArmTest();

 private:

  ctre::phoenix::motorcontrol::can::TalonFX m_left{ deviceIDs::kLeftArmMotorID };
  ctre::phoenix::motorcontrol::can::TalonFX m_right{ deviceIDs::kRightArmMotorID }; 

  AbsoluteEncoder m_enc{ deviceIDs::kArmEncoderID, physical::kArmAbsoluteOffset, -1 };

  double kS = 0.0;
  double kG = 0.0;
  double kV = 0.0;
  double kA = 0.0;

  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;

  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints{ 360_deg_per_s, 360_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_goal;
  frc::TrapezoidProfile<units::degrees>::State m_setpoint{ 0_deg, 0_deg_per_s };

  units::second_t dt = 20_ms;
  units::degree_t m_angle;
};
