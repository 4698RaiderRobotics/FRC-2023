// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "commands/OpenGrabber.h"

#if defined(Claw)
OpenGrabber::OpenGrabber( GrabberSubsystem *grabber, double speed ) 
        : m_grabber{ grabber }, m_speed{ speed } {
  AddRequirements( { grabber } );
}

void OpenGrabber::Initialize() {
  m_grabber->Spin( -m_speed );
  
  m_grabber->Open();
}

void OpenGrabber::Execute() {}

bool OpenGrabber::IsFinished() {
  return true;
}
#endif