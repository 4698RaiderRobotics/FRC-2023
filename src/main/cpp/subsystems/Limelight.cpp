#include "subsystems/Limelight.h"

Limelight::Limelight() {
    frc::SmartDashboard::PutData("Field2", &m_field);

}
// Targets the robot based on the input from the limelight camera
frc::ChassisSpeeds Limelight::TargetRobot( void ) {
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber("ty",0.0);

    t_speeds.vy = 0_mps;
    t_speeds.vx = -targetY * kxP * physical::kMaxDriveSpeed;
    t_speeds.omega = -targetX * kOmegaP * physical::kMaxTurnSpeed;

    frc::SmartDashboard::PutNumber( "Target X Speed", t_speeds.vx.value() );
    frc::SmartDashboard::PutNumber( "Target Omega Speed", t_speeds.omega.value() );

    return t_speeds;
}

frc::ChassisSpeeds Limelight::TargetRobot_AT( void ) {
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber( "ty", 0.0 );
    // [x,y,z,pitch,yaw,roll]
    std::vector<double> defaultValue{};
    camtran = table->GetNumberArray("camtran", defaultValue);
    t_speeds.vx = -targetY * kxP * physical::kMaxDriveSpeed;
    t_speeds.vy = camtran[0]*  kyP * physical::kMaxDriveSpeed;
    t_speeds.omega = -targetX * kOmegaP * physical::kMaxTurnSpeed;
    
    return t_speeds;
}
// Sets the limelight to new targeting settings
void Limelight::SetPipeline( int pipelineId ) {
    table->PutNumber( "pipeline", pipelineId );
}

frc::Pose2d Limelight::VisionPose( void ) {
    std::vector<double> defaultValue{};
    //Translation(x,y,z)Rotation(x,y,z)
    botpose = table->GetNumberArray("botpose", defaultValue);
    rZ = units::degree_t{ botpose[5] };
    m_field.SetRobotPose(frc::Pose2d(units::meter_t{botpose[0]},units::meter_t{botpose[1]}, frc::Rotation2d { rZ } ));
    return frc::Pose2d(units::meter_t{botpose[0]},units::meter_t{botpose[1]}, frc::Rotation2d { rZ } );
}