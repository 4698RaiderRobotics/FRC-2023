#include "commands/UpdateOdom.h"

UpdateOdom::UpdateOdom(Drivetrain *drive, Limelight *limelight)
    : m_drive{ drive }, m_limelight{ limelight } {
    AddRequirements( { drive, limelight } );
}
void UpdateOdom::Execute() {
    frc::Pose2d AP_Pose;
    if (m_limelight->VisionPose(&AP_Pose)) {
        m_drive->ResetPose(AP_Pose);
    }
}