// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>
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

  void GotoAngle(units::degree_t armAngle, units::degree_t wristAngle);

  void AdjustArmAngle(units::degree_t delta_angle);

  void AdjustWristAngle( units::degree_t delta_angle);

  void ArmOn( double armSpeed, double wristSpeed );

  bool Finished( );

  void ArmDataSetup( );

  void ArmDataUpdate();

  void ArmData();

  units::degree_t GetArmAngle();

  units::degree_t GetWristAngle();

 private:

   ctre::phoenix::motorcontrol::can::TalonFX m_leftArm{ deviceIDs::kLeftArmMotorID };
   ctre::phoenix::motorcontrol::can::TalonFX m_rightArm{ deviceIDs::kRightArmMotorID };

  rev::CANSparkMax m_wrist{ deviceIDs::kWristMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

  rev::SparkMaxRelativeEncoder m_enc{ m_wrist.GetEncoder() };

  AbsoluteEncoder m_armEncoder{ deviceIDs::kArmEncoderID, physical::kArmAbsoluteOffset, true };
  AbsoluteEncoder m_wristEncoder{ deviceIDs::kWristEncoderID, physical::kWristAbsoluteOffset, true };

  units::degree_t armAngle;
  units::degree_t wristAngle;

  units::degree_t max_angle = 120_deg;
  units::degree_t min_angle = -135_deg;
  units::degree_t maxWristAngle = 75_deg;
  units::degree_t minWristAngle = -180_deg;

  frc2::PIDController wristPID{ pidf::kWristP, pidf::kWristI, pidf::kWristD };

  frc::ArmFeedforward wristFeedforward{ units::volt_t{ pidf::kWristS }, units::volt_t{ pidf::kWristG },  units::unit_t<frc::ArmFeedforward::kv_unit> { pidf::kWristV } };

  frc::TrapezoidProfile<units::degrees>::Constraints m_wristConstraints{ 180_deg_per_s, 360_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_wristGoal;
  frc::TrapezoidProfile<units::degrees>::State m_wristSetpoint;

  frc2::PIDController armPID{ pidf::kArmP, pidf::kArmI, pidf::kArmD };

  frc::ArmFeedforward armFeedforward{ units::volt_t{ pidf::kArmS }, units::volt_t{ pidf::kArmG },  units::unit_t<frc::ArmFeedforward::kv_unit> { pidf::kArmV } };

  frc::TrapezoidProfile<units::degrees>::Constraints m_armConstraints{ 180_deg_per_s, 360_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint;

  units::second_t dt = 20_ms;
  units::degree_t m_armAngleGoal;
  units::degree_t m_wristAngleGoal;

  bool disabled = true;
};
