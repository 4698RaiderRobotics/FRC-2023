#include "commands/Intake.h"

#include <frc/Timer.h>

Intake::Intake(GrabberSubsystem *grabber, bool GamePieceType) 
      : m_grabber{ grabber}, m_GamePieceType{ GamePieceType } {
  AddRequirements( { grabber } );
}

// Called when the command is initially scheduled.
void Intake::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();

  if (m_GamePieceType) {
    m_grabber->Spin(0.2);
  }
  else {
    m_grabber->Spin(-0.2);
  }
}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  m_grabber->Spin(0.0);
}

// Returns true when the command should end.
bool Intake::IsFinished() {
  //There is an initial 30 A spike that lasts for ~150 ms so...
  return (m_grabber->GetCurrent() > 4_A) & (frc::Timer::GetFPGATimestamp() - m_startTime > 0.3_s);
}
