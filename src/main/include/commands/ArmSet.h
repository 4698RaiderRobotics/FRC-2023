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
   ArmSet(ArmSubsystem* arm, units::degree_t armAngle, units::degree_t wristAngle, double delayProportion);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

  Command::InterruptionBehavior GetInterruptionBehavior() const override;

 private:
  ArmSubsystem *m_arm; 
  units::degree_t m_armAngle;
  units::degree_t m_wristAngle;
  units::degree_t m_delayArmAngle;
  units::degree_t m_delayWristAngle;
  double m_delayProportion;

  units::degree_t m_startingArmAngle;
  units::degree_t m_startingWristAngle;

  units::second_t m_startTime;
};
