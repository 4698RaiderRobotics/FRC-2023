// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/ArmSubsystem.h"

class ArmSet
    : public frc2::CommandHelper<frc2::CommandBase, ArmSet> {
 public:
  ArmSet( units::degree_t angle, ArmSubsystem *arm, bool isCone = false );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  units::degree_t m_angle;
  ArmSubsystem *m_arm; 
};
