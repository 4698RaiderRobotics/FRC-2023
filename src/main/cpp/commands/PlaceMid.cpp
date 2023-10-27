// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceMid.h"
#include <frc2/command/ScheduleCommand.h>

PlaceMid::PlaceMid( ArmSubsystem *arm, GrabberSubsystem *grabber ) 
 : m_arm{ arm }, m_grabber{ grabber } {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements( { m_arm } );
}

// Called when the command is initially scheduled.
void PlaceMid::Initialize() {
  fmt::print("PlaceMid::Initialize()\n");

  delete m_armsetcmd;
  m_armsetcmd = nullptr;

  if (m_grabber->HasCone()) {
    fmt::print("HasCone\n");
    m_armsetcmd = new ArmSet(m_arm, physical::kArmConeMidPlaceHeight, physical::kWristConeMidPlaceHeight, 0.3, true);
  } else if (m_grabber->HasCube()) {
    fmt::print("HasCube\n");
    m_armsetcmd = new ArmSet(m_arm, physical::kArmCubeMidPlaceHeight, physical::kWristCubeMidPlaceHeight, 0.3, true);
  } else {
    fmt::print("HasNothing\n");
    m_armsetcmd = new ArmSet(m_arm, physical::kArmSubstationMidPlaceHeight, physical::kWristSubstationMidPlaceHeight, 0.3, true);
  }

  frc2::CommandScheduler::GetInstance().Schedule( m_armsetcmd );
}
