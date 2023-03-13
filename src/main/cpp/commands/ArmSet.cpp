// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "commands/ArmSet.h"


ArmSet::ArmSet( ArmSubsystem *arm, units::degree_t angle )
        : m_arm{ arm }, m_angle{ angle } {
  AddRequirements( { arm } );
}

void ArmSet::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_arm->Brake( false );
}

void ArmSet::Execute() {
  m_arm->GotoAngle( m_angle );
}

bool ArmSet::IsFinished() {
  return m_arm->Finished( ) || frc::Timer::GetFPGATimestamp() - m_startTime > 2_s;
}
