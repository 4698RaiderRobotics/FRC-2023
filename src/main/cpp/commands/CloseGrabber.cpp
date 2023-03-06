// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CloseGrabber.h"

CloseGrabber::CloseGrabber( GrabberSubsystem *grabber, bool spin_on_close )
        : m_grabber{ grabber }, m_spin_on_close{spin_on_close} {
  AddRequirements( { grabber } );
}

// Called when the command is initially scheduled.
void CloseGrabber::Initialize() {
  if( m_spin_on_close ) {
    m_grabber->Spin( m_grabber->kRollerGripPercent );
  }
}

// Called repeatedly when this Command is scheduled to run
void CloseGrabber::Execute() {
  m_grabber->Close();
}

// Called once the command ends or is interrupted.
void CloseGrabber::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool CloseGrabber::IsFinished() {
  return true;
}
