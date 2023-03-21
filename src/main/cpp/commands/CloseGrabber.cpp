// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CloseGrabber.h"
#if defined(Claw)
CloseGrabber::CloseGrabber( GrabberSubsystem *grabber, bool spin_on_close )
        : m_grabber{ grabber }, m_spin_on_close{spin_on_close} {
  AddRequirements( { grabber } );
}

void CloseGrabber::Initialize() {
  if( m_spin_on_close ) {
    #if defined(Claw)
    m_grabber->Spin( m_grabber->kRollerGripPercent );
    #endif
  }
}

void CloseGrabber::Execute() {
  #if defined(Claw)
  m_grabber->Close();
  #endif
}

bool CloseGrabber::IsFinished() {
  return true;
}
#endif