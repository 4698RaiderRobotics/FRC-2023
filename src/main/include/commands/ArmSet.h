// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/time.h>

#include "subsystems/ArmSubsystem.h"

class ArmSet
    : public frc2::CommandHelper<frc2::CommandBase, ArmSet> {
 public:
  ArmSet( ArmSubsystem *arm, units::degree_t angle );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  ArmSubsystem *m_arm; 
  units::degree_t m_angle;

  units::second_t m_startTime;
};
