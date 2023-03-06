// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

class Drivetrain;

class TestProfileMove
    : public frc2::CommandHelper<frc2::CommandBase,TestProfileMove> {
 public:
  enum Direction { FORWARD, LEFT };

  TestProfileMove( Drivetrain *drive, units::meter_t distance, Direction d );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

private:
  Drivetrain* m_drive;
  units::meter_t m_distance;
  Direction m_direction;
  units::second_t m_elapsed_time;

  frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{ 1.5_mps, 1.5_mps_sq };

  frc::TrapezoidProfile<units::meters>::State m_Goal;
  frc::TrapezoidProfile<units::meters>::State m_Setpoint;

  frc::TrapezoidProfile<units::meters> m_profile{ m_linearConstraints, m_Goal, m_Setpoint };
};
