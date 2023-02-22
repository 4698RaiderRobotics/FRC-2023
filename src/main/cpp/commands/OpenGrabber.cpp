// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OpenGrabber.h"

OpenGrabber::OpenGrabber( GrabberSubsystem *grabber ) 
        : m_grabber{ grabber } {
  AddRequirements( { grabber } );
}

// Called when the command is initially scheduled.
void OpenGrabber::Initialize() {
  m_grabber->Open();
}

// Called repeatedly when this Command is scheduled to run
void OpenGrabber::Execute() {}

// Called once the command ends or is interrupted.
void OpenGrabber::End(bool interrupted) {}

// Returns true when the command should end.
bool OpenGrabber::IsFinished() {
  return true;
}
