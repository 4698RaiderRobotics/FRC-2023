// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "commands/ArmSet.h"


ArmSet::ArmSet(ArmSubsystem* arm, units::degree_t armAngle, units::degree_t wristAngle, double delayProportion)
  : m_arm{ arm }, m_armAngle{ armAngle }, m_wristAngle{ wristAngle }, m_delayProportion{ delayProportion } {
  AddRequirements( { arm } );
}

void ArmSet::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_startingArmAngle = m_arm->GetArmAngle();
  m_startingWristAngle = m_arm->GetWristAngle();
  m_delayArmAngle = (m_armAngle - m_startingArmAngle) * m_delayProportion;
  m_delayWristAngle = (m_wristAngle - m_startingWristAngle) * m_delayProportion;
}

void ArmSet::Execute() {
  fmt::print("ArmSet::Execute()\n");
  if (m_delayArmAngle > 0_deg && m_arm->GetArmAngle() < m_startingArmAngle + m_delayArmAngle) {
    m_arm->GotoAngle(m_armAngle, m_startingWristAngle);
  }
  else if (m_delayArmAngle < 0_deg && m_arm->GetWristAngle() < m_startingWristAngle + m_delayWristAngle) {
    fmt::print("Delay Arm: = {}\n", m_arm->GetWristAngle());
    m_arm->GotoAngle(m_startingArmAngle, m_wristAngle);
  }
  else {
    fmt::print("Normal: = {}\n", m_arm->GetArmAngle());
    m_arm->GotoAngle(m_armAngle, m_wristAngle);
  }
  // m_arm->GotoAngle(m_armAngle, m_wristAngle);
}

bool ArmSet::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - m_startTime > 2_s;
}

void ArmSet::End(bool interrupted) {
  fmt::print("ArmSset::End - interrupted {}\n", interrupted);
}

frc2::Command::InterruptionBehavior ArmSet::GetInterruptionBehavior() const {
  return frc2::Command::InterruptionBehavior::kCancelIncoming;
}