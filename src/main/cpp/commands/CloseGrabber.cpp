// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CloseGrabber.h"

CloseGrabber::CloseGrabber( GrabberSubsystem *grabber )
        : m_grabber{ grabber } {
  AddRequirements( { grabber } );
}

// Called when the command is initially scheduled.
void CloseGrabber::Initialize() {
  m_grabber->Spin( 0.25 );
  m_grabber->Close();
}

// Called repeatedly when this Command is scheduled to run
void CloseGrabber::Execute() {
  m_grabber->Spin( 0.1 );
}

// Called once the command ends or is interrupted.
void CloseGrabber::End(bool interrupted) {
  
}

// Returns true when the command should end.
bool CloseGrabber::IsFinished() {
  return true;
}
