
#include "commands/TargetLimelight.h"

TargetLimelight::TargetLimelight( Drivetrain* drive, Limelight* limelight, bool AprilTag )
    : m_drive{ drive }, m_limelight{ limelight }, apriltag{AprilTag} {
        AddRequirements( { drive, limelight } );
    }

void TargetLimelight::Execute() {
    if(apriltag) {
        m_drive->Drive(m_limelight->TargetRobot_AT(),false);
    }
    else {
        m_drive->Drive( m_limelight->TargetRobot(), false );
    }
}