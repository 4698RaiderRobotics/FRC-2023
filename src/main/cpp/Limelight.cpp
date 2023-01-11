#include "Limelight.h"

// Targets the robot based on the input from the limelight camera
frc::ChassisSpeeds Limelight::TargetRobot( void ) {
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber("ty",0.0);
    
    t_speeds.vx = -targetY * kxP * physical::kMaxDriveSpeed;
    t_speeds.omega = -targetX * kOmegaP * physical::kMaxTurnSpeed;

    frc::SmartDashboard::PutNumber( "Target X Speed", t_speeds.vx.value() );
    frc::SmartDashboard::PutNumber( "Target Omega Speed", t_speeds.omega.value() );

    return t_speeds;
}

// Sets the limelight to new targeting settings
void Limelight::SetPipeline( int pipelineId ) {
    table->PutNumber( "pipeline", pipelineId );
}