#include "commands/UpdateOdom.h"

UpdateOdom::UpdateOdom(Drivetrain *drive, Limelight *limelight)
    : m_drive{ drive }, m_limelight{ limelight } {
    AddRequirements( { drive, limelight } );
}
void UpdateOdom::Execute() {
    m_drive->ResetPose( m_limelight->VisionPose() );
}