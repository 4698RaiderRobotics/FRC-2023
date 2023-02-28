#include "subsystems/Limelight.h"
#include <iostream>

Limelight::Limelight()
{
    frc::SmartDashboard::PutData("Field2", &m_field);
}

void Limelight::Periodic( void  ) {
    frc::Pose2d pose;

    TargetRobot_AT( pose );
    frc::SmartDashboard::PutNumber( "Current X Pose", pose.X().value() );
    frc::SmartDashboard::PutNumber( "Current Y Pose", -pose.Y().value() );
    frc::SmartDashboard::PutNumber( "Current Omega Pose", -pose.Rotation().Degrees().value() );

}

bool Limelight::TargetRobot_AT( frc::Pose2d &april_tag_pose)
{
    camtran = table->GetNumberArray("camerapose_targetspace", defaultValue);
    if ( camtran.size() == 0 ) {
        return false;
    }

    if( table->GetNumber("tv", 0.0 ) < 1.0 ) {
        return false;
    }
    
    double x_val{0}, y_val{0}, o_val{0};

    for( int i=0; i<5; ++i ) {
        camtran = table->GetNumberArray("camerapose_targetspace", defaultValue);
        x_val += camtran[2];
        y_val += camtran[0];
        o_val += camtran[4];
    }
    april_tag_pose = frc::Pose2d( units::meter_t{ x_val / 5.0 },
                                  units::meter_t{ y_val / 5.0 },
                                  frc::Rotation2d{ units::degree_t{ o_val / 5.0 } } );

    return true;
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