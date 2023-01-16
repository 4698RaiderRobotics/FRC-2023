#include "commands/VisionPose.h"

VisionPose::VisionPose( Limelight *limelight)
    : m_limelight{ limelight } {
        AddRequirements( {limelight} );
    }

void VisionPose::FixPose() {
    
}