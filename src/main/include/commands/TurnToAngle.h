// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_acceleration.h>
#include <units/time.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurnToAngle
    : public frc2::CommandHelper<frc2::CommandBase, TurnToAngle> {
 public:
  TurnToAngle( Drivetrain *drive, units::degree_t angle );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
  Drivetrain *m_drive;
  units::degree_t m_angle;

  units::second_t dt = 20_ms;

  bool m_finished = false;

  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints{ 120_deg_per_s, 180_deg_per_s_sq };
  frc::TrapezoidProfile<units::degrees>::State m_goal;
  frc::TrapezoidProfile<units::degrees>::State m_setpoint;
};
