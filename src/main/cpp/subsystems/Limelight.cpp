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
    t_speeds.vx = -targetY * pidf::kXTargetP * physical::kMaxDriveSpeed;
    // Go left if off center to the right
    t_speeds.vy = camtran[0]*  pidf::kYTargetP * physical::kMaxDriveSpeed;
    // Rotate if the target isn't centered
    t_speeds.omega = -targetX * pidf::kOmegaTargetP * physical::kMaxTurnSpeed;
    
    return t_speeds;
}

bool Limelight::Targeted( void ) {
    return ( targetX && targetY && camtran[0] ) < 0.1;
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