
#include "commands/TargetLimelight.h"

TargetLimelight::TargetLimelight( Drivetrain* drive, Limelight* limelight )
    : m_drive{ drive }, m_limelight{ limelight } {
        AddRequirements( { drive, limelight } );
    }

void TargetLimelight::Execute() {
    m_drive->Drive( m_limelight->TargetRobot(), false );
}