#include "subsystems/Limelight.h"
#include <iostream>

Limelight::Limelight()
{
    frc::SmartDashboard::PutData("Field2", &m_field);
    frc::SmartDashboard::PutNumber( "xP", kXTargetP );
    frc::SmartDashboard::PutNumber( "yP", kYTargetP );
    frc::SmartDashboard::PutNumber( "omegaP", kOmegaTargetP );
}

frc::ChassisSpeeds Limelight::TargetRobot_AT(void)
{
    double x = frc::SmartDashboard::GetNumber( "xP", 0.0 );
    double y = frc::SmartDashboard::GetNumber( "yP", 0.0 );
    double omega = frc::SmartDashboard::GetNumber( "omegaP", 0.0 );
    if ( kXTargetP != x ) { kXTargetP = x; }
    if ( kYTargetP != y ) { kYTargetP = y; }
    if ( kOmegaTargetP != omega ) { kOmegaTargetP = omega; }
    targetX = table->GetNumber("tx", 0.0);
    targetY = table->GetNumber("ty", 0.0);
    // [x,y,z,pitch,yaw,roll]
    frc::SmartDashboard::PutNumber( "tx", targetX );
    frc::SmartDashboard::PutNumber( "ty", targetY );

    camtran = table->GetNumberArray("camerapose_targetspace", defaultValue);
    if ( camtran.size() == 0 ) {
        return t_speeds = { 0_mps, 0_mps, 0_rad_per_s };
    }
    
    if ( camtran.size() > 0 ) {
        frc::SmartDashboard::PutNumber( "Yaw", camtran[0]);
    }
    // Back up if the target is high
    t_speeds.vx = targetY * kXTargetP * physical::kMaxDriveSpeed;
    // Go left if off center to the right
    t_speeds.vy = camtran[0] * kYTargetP * physical::kMaxDriveSpeed;
    // Rotate if the target isn't centered
    t_speeds.omega = -targetX * kOmegaTargetP * physical::kMaxTurnSpeed;

    return t_speeds;
}

bool Limelight::Finished(void)
{
    if ( camtran.size() == 0 ) {
        std::cout << "no camtran";
        return true;
    }
    return (targetX < physical::kLimelightTargetError && targetY < physical::kLimelightTargetError && camtran[0] < physical::kLimelightTargetError);
}

// Sets the limelight to new targeting settings
void Limelight::SetPipeline(int pipelineId)
{
    table->PutNumber("pipeline", pipelineId);
}

bool Limelight::VisionPose(frc::Pose2d* AP_Pose)
{

    // Translation(x, y, z)Rotation(pitch, yaw, roll)
    botpose = table->GetNumberArray("botpose", defaultValue);
    if (botpose.size() == 0) {
        return false;
    }
    rZ = units::degree_t{botpose[5]};
    *AP_Pose = frc::Pose2d(units::meter_t{botpose[0] + 8.26}, units::meter_t{botpose[1] + 4.01}, frc::Rotation2d{rZ});
    // m_field.SetRobotPose(p_Pose);
    // convert between limelight's center origin co-ordinats to wpi's bottom left corner origin
    //  field is 8.02m x 16.54m
    double ll_Pose[]{
        (AP_Pose->X().value()),
        (AP_Pose->Y().value()),
        AP_Pose->Rotation().Degrees().value()};
    frc::SmartDashboard::PutNumberArray("l_Pose", ll_Pose);
    return true;
}

void Limelight::LimelightTest( ) {
    frc::SmartDashboard::PutNumber( "tx", targetX );
    frc::SmartDashboard::PutNumber( "ty", targetY );
    if ( camtran.size() > 0 ) {
        frc::SmartDashboard::PutNumber( "Yaw", camtran[0]);
    }
}