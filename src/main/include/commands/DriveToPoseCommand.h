// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveToPoseCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveToPoseCommand> {
 public:
  DriveToPoseCommand(  Drivetrain* );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

private:
  Drivetrain *m_drive;
  frc::Pose2d m_targetpose;
  units::second_t m_profile_time;

  frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{ 3_mps, 0.5_mps_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_omegaConstraints{ 90_deg_per_s, 10_deg_per_s_sq };
  
  frc::TrapezoidProfile<units::meters> m_xProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::meters> m_yProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::degrees> m_omegaProfile{ m_omegaConstraints, {0.0_deg}, {0.0_deg} };
};
