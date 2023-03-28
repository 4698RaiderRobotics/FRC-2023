#include "commands/Intake.h"
#if !defined(Claw)
#include <frc/Timer.h>

Intake::Intake(GrabberSubsystem *grabber, bool reversed ) 
      : m_grabber{ grabber}, m_reversed{ reversed } {
  AddRequirements( { grabber } );
}

// Called when the command is initially scheduled.
void Intake::Initialize() {
  
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_reversed ? m_grabber->Spin( -0.5 ) : m_grabber->Spin( 0.5 );
  fmt::print( "Intake::Initialize {}\n", m_reversed );
}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {
  if ( ( m_grabber->GetCurrent() > 20_A) && (frc::Timer::GetFPGATimestamp() - m_startTime > 0.5_s ) ) {
    m_grabber->Spin( 0.0 );
  }
}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  m_grabber->Spin( 0.0 );
}

// Returns true when the command should end.
bool Intake::IsFinished() {

  //There is an initial 30 A spike that lasts for ~150 ms so...
  //return ( m_grabber->GetCurrent() > 20_A) & (frc::Timer::GetFPGATimestamp() - m_startTime > 0.5_s );
  return false;
}
#endif