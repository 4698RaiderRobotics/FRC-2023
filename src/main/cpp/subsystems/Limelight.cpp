#include "subsystems/Limelight.h"

Limelight::Limelight() {
    frc::SmartDashboard::PutData("Field2", &m_field);

}

frc::ChassisSpeeds Limelight::TargetRobot_AT( void ) {
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber( "ty", 0.0 );
    // [x,y,z,pitch,yaw,roll]
    
    camtran = table->GetNumberArray("camtran", defaultValue);
    // Back up if the target is high
    t_speeds.vx = -targetY * kxP * physical::kMaxDriveSpeed;
    // Go left if off center to the right
    t_speeds.vy = camtran[0]*  kyP * physical::kMaxDriveSpeed;
    // Rotate if the target isn't centered
    t_speeds.omega = -targetX * kOmegaP * physical::kMaxTurnSpeed;
    
    return t_speeds;
}

// Sets the limelight to new targeting settings
void Limelight::SetPipeline( int pipelineId ) {
    table->PutNumber( "pipeline", pipelineId );
}

frc::Pose2d Limelight::VisionPose( void ) {
    
    // Translation(x, y, z)Rotation(pitch, yaw, roll)
    botpose = table->GetNumberArray("botpose", defaultValue);
    rZ = units::degree_t{ botpose[5] };
    m_field.SetRobotPose(frc::Pose2d(units::meter_t{botpose[0]},units::meter_t{botpose[1]}, frc::Rotation2d { rZ } ));
    return frc::Pose2d(units::meter_t{botpose[0]},units::meter_t{botpose[1]}, frc::Rotation2d { rZ } );
}