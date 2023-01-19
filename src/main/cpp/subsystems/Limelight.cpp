#include "subsystems/Limelight.h"

Limelight::Limelight()
{
    frc::SmartDashboard::PutData("Field2", &m_field);
}

frc::ChassisSpeeds Limelight::TargetRobot_AT(void)
{
    targetX = table->GetNumber("tx", 0.0);
    targetY = table->GetNumber("ty", 0.0);
    // [x,y,z,pitch,yaw,roll]

    camtran = table->GetNumberArray("camtran", defaultValue);
    // Back up if the target is high
    t_speeds.vx = -targetY * pidf::kXTargetP * physical::kMaxDriveSpeed;
    // Go left if off center to the right
    t_speeds.vy = camtran[0] * pidf::kYTargetP * physical::kMaxDriveSpeed;
    // Rotate if the target isn't centered
    t_speeds.omega = -targetX * pidf::kOmegaTargetP * physical::kMaxTurnSpeed;

    return t_speeds;
}

bool Limelight::Targeted(void)
{
    return (targetX && targetY && camtran[0]) < 0.1;
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