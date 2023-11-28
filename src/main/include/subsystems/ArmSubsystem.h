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
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

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
  // AbsoluteEncoder m_wristEncoder{ deviceIDs::kWristEncoderID, physical::kWristAbsoluteOffset, true };

  ctre::phoenix::sensors::CANCoder m_wristEncoder{ 20 };

  units::degree_t armAngle;
  units::degree_t wristAngle;

  units::degree_t max_angle = 120_deg;
  units::degree_t min_angle = -135_deg;
  units::degree_t maxWristAngle = 75_deg;
  units::degree_t minWristAngle = -180_deg;

  frc2::PIDController wristPID{ pidf::kWristP, pidf::kWristI, pidf::kWristD };

  frc::ArmFeedforward wristFeedforward{ units::volt_t{ pidf::kWristS }, units::volt_t{ pidf::kWristG }, 
                                        units::unit_t<frc::ArmFeedforward::kv_unit> { pidf::kWristV }, 
                                        units::unit_t<frc::ArmFeedforward::ka_unit> { pidf::kWristA } };

//  frc::TrapezoidProfile<units::degrees>::Constraints m_wristConstraints{ 360_deg_per_s, 720_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_wristConstraints{ 360_deg_per_s, 720_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_wristGoal;
  frc::TrapezoidProfile<units::degrees>::State m_wristSetpoint;

  frc2::PIDController armPID{ pidf::kArmP, pidf::kArmI, pidf::kArmD };

  frc::ArmFeedforward armFeedforward{ units::volt_t{ pidf::kArmS }, units::volt_t{ pidf::kArmG }, 
                                      units::unit_t<frc::ArmFeedforward::kv_unit> { pidf::kArmV }, 
                                      units::unit_t<frc::ArmFeedforward::ka_unit> { pidf::kArmA } };

//  frc::TrapezoidProfile<units::degrees>::Constraints m_armConstraints{ 360_deg_per_s, 540_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_armConstraints{ 360_deg_per_s, 540_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint;

  frc::Mechanism2d m_mech{100/39.0, 100/39.0};

  frc::MechanismRoot2d* m_root = m_mech.GetRoot("shoulder", 50/39.0, 50/39.0);

  frc::MechanismLigament2d* m_armLigament = m_root->Append<frc::MechanismLigament2d>("arm", 25.5/39.0, 90_deg);

  frc::MechanismLigament2d* m_wristLigament = m_armLigament->Append<frc::MechanismLigament2d>("wrist", 11.25/39.0, 90_deg);
  frc::MechanismLigament2d* m_wristLigament2 = m_wristLigament->Append<frc::MechanismLigament2d>("wrist2", 12.5/39.0, 30_deg);

  frc::MechanismLigament2d* m_armSetpointLigament = m_root->Append<frc::MechanismLigament2d>("armSetpoint", 25.5/39.0, 90_deg, 6, 
                                                                                              frc::Color8Bit{frc::Color::kBlue});

  frc::MechanismLigament2d* m_wristSetpointLigament = m_armSetpointLigament->Append<frc::MechanismLigament2d>("wristSetpoint", 11.25/39.0, 90_deg, 6, 
                                                                                                              frc::Color8Bit{ frc::Color::kBlue });
  frc::MechanismLigament2d* m_wristSetpointLigament2 = m_wristSetpointLigament->Append<frc::MechanismLigament2d>("wristSetpoint2", 12.5/39.0, 30_deg, 6,
                                                                                                                  frc::Color8Bit{ frc::Color::kBlue });

  units::second_t dt = 20_ms;
  units::degree_t m_armAngleGoal;
  units::degree_t m_wristAngleGoal;

  bool disabled = true;
};
