// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "commands/ArmSet.h"


ArmSet::ArmSet(ArmSubsystem* arm, units::degree_t armAngle, units::degree_t wristAngle, double delayProportion, bool finish)
  : m_arm{ arm }, m_armAngle{ armAngle }, m_wristAngle{ wristAngle }, m_delayProportion{ delayProportion }, m_finish{ finish } {
  AddRequirements({ arm });

  if (m_delayProportion > 1.0) {
    m_delayProportion = 1.0;
  }
}

void ArmSet::Initialize() {
  m_isWaiting = false;
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_startingArmAngle = m_arm->GetArmAngle();
  m_startingWristAngle = m_arm->GetWristAngle();
  m_delayArmAngle = (m_armAngle - m_startingArmAngle) * m_delayProportion;
  m_delayWristAngle = (m_wristAngle - m_startingWristAngle) * m_delayProportion;
  // fmt::print("ArmSet::Initialize() Arm goal({}), current({}), Wrist goal({}), current({}), time {}\n", m_armAngle, m_startingArmAngle,
  //   m_wristAngle, m_startingWristAngle, frc::Timer::GetFPGATimestamp());
  // fmt::print("   ArmDelayAngle({}), WristDelayAngle({})\n", m_delayArmAngle, m_delayWristAngle);

  if (m_delayProportion > 0) {
    if (m_delayArmAngle > 0_deg /* && m_arm->GetArmAngle() < m_startingArmAngle + m_delayArmAngle */) {
      // fmt::print("Wrist Delayed until arm at {}, current {}, time {}\n", m_startingArmAngle + m_delayArmAngle, m_arm->GetArmAngle(), frc::Timer::GetFPGATimestamp());
      m_arm->GotoAngle(m_armAngle, m_startingWristAngle);
      m_isWaiting = true;
    }
    else if (m_delayArmAngle < 0_deg /* && m_arm->GetWristAngle() < m_startingWristAngle + m_delayWristAngle */) {
      // fmt::print("Arm Delayed until wrist at {}, current {}, time {}\n", m_startingWristAngle + m_delayWristAngle, m_arm->GetWristAngle(), frc::Timer::GetFPGATimestamp());
      m_arm->GotoAngle(m_startingArmAngle, m_wristAngle);
      m_isWaiting = true;
    }
  } else {
    // fmt::print("Normal Goto Angle\n");
    m_arm->GotoAngle(m_armAngle, m_wristAngle);
  }

}

void ArmSet::Execute() {

  if (m_delayArmAngle > 0_deg && m_arm->GetArmAngle() < m_startingArmAngle + m_delayArmAngle) {
    /* do Nothing.... wait for arm */
    //  fmt::print("Wrist Delay, arm_ang({}), delay_ang({})\n", m_arm->GetArmAngle(), m_startingArmAngle + m_delayArmAngle);
  }
  else if (m_delayArmAngle < 0_deg && m_arm->GetWristAngle() < m_startingWristAngle + m_delayWristAngle) {
    /* do nothing ... wait for wrist */
    //  fmt::print("Arm Delay, wrist_ang({}), delay_ang({})\n", m_arm->GetWristAngle(), m_startingWristAngle + m_delayWristAngle);
  }
  else if (m_isWaiting) {
    /* Switch to moving both angles */
    // fmt::print("Normal Goto Angle, Arm goal({}), current({}), Wrist goal({}), current({}), time {}\n", m_armAngle, m_arm->GetArmAngle(),
    //   m_wristAngle, m_arm->GetWristAngle(), frc::Timer::GetFPGATimestamp());
    m_arm->GotoAngle(m_armAngle, m_wristAngle);
    m_isWaiting = false;
  }

  //fmt::print("ArmSet::Execute()\n");
  // if (m_delayArmAngle > 0_deg && m_arm->GetArmAngle() < m_startingArmAngle + m_delayArmAngle) {
  // fmt::print("Wrist Delay, ang{}, dang{}\n", m_arm->GetArmAngle(), m_startingArmAngle + m_delayArmAngle);
  //   m_arm->GotoAngle(m_armAngle, m_startingWristAngle);
  // }
  // else if (m_delayArmAngle < 0_deg && m_arm->GetWristAngle() < m_startingWristAngle + m_delayWristAngle) {
  //   fmt::print("Delay Arm, ang{}, dang{}\n", m_arm->GetWristAngle(), m_startingWristAngle + m_delayWristAngle);
  //   m_arm->GotoAngle(m_startingArmAngle, m_wristAngle);
  // }
  // else {
  //   fmt::print("Normal\n");
  //   m_arm->GotoAngle(m_armAngle, m_wristAngle);
  // }

  // m_arm->GotoAngle(m_armAngle, m_wristAngle);
}

bool ArmSet::IsFinished() {
  if( m_isWaiting ) return false;

  if (!m_finish) {
    return units::math::abs( m_armAngle - m_arm->GetArmAngle() ) < 6_deg && units::math::abs( m_wristAngle - m_arm->GetWristAngle() ) < 6_deg;
  } else {
    return (units::math::abs(m_armAngle - m_arm->GetArmAngle()) < 6_deg && units::math::abs(m_wristAngle - m_arm->GetWristAngle()) < 6_deg) ||
            (units::math::abs( frc::Timer::GetFPGATimestamp() - m_startTime ) > 1_s);
  }
  // if (m_finish && !m_isWaiting) {
  //   return true;
  // } else {
  //   return units::math::abs(m_armAngle - m_arm->GetArmAngle()) < 6_deg && units::math::abs(m_wristAngle - m_arm->GetWristAngle()) < 6_deg;
  // }
  //  return frc::Timer::GetFPGATimestamp() - m_startTime > 10_s;
}

void ArmSet::End(bool interrupted) {
  // fmt::print("ArmSset::End - interrupted {}\n", interrupted);
}

frc2::Command::InterruptionBehavior ArmSet::GetInterruptionBehavior() const {
  return frc2::Command::InterruptionBehavior::kCancelSelf;
}