
#include <iostream>

#include <frc/Timer.h>
#include <frc/DataLogManager.h>

#include "subsystems/Limelight.h"

Limelight::Limelight()
{
    m_poseLogEntry = wpi::log::DoubleArrayLogEntry( frc::DataLogManager::GetLog(), "/ATVision/Robot2D" );

//    frc::SmartDashboard::PutData("Field2", &m_field);
}

void Limelight::Periodic( void  ) {
    // frc::Pose2d pose;

    // TargetRobot_AT( pose );
    // frc::SmartDashboard::PutNumber( "Current X Pose", pose.X().value() );
    // frc::SmartDashboard::PutNumber( "Current Y Pose", -pose.Y().value() );
    // frc::SmartDashboard::PutNumber( "Current Omega Pose", -pose.Rotation().Degrees().value() );

}

// Unused
bool Limelight::TargetRobot_AT_Unused( frc::Pose2d &april_tag_pose)
{
    if ( !haveValidAprilTag() ) {
        return false;
    }

    std::vector<double> camtran{};

    camtran = table->GetNumberArray("camerapose_targetspace", defaultValue);
    if( camtran.size() < 6 ) return false;
 
    april_tag_pose = frc::Pose2d( units::meter_t{ camtran[2] },
                                  units::meter_t{ camtran[0] },
                                  frc::Rotation2d{ units::degree_t{ camtran[4] } } );

    return true;
}

// Gets the current pose of the robot in field space
bool Limelight::getFieldAprilTagPose( frc::Pose2d&april_tag_pose, units::second_t &timestamp ) {
    if ( !haveValidAprilTag() ) {
        return false;
    }

    std::vector<double> botpose{};
    botpose = table->GetNumberArray("botpose", defaultValue);
    if( botpose.size() < 6 ) return false;

    april_tag_pose = frc::Pose2d( units::meter_t{ botpose[0] } + physical::kLimelightXAxisOffset,
                                  units::meter_t{ botpose[1] } + physical::kLimelightYAxisOffset,
                                  frc::Rotation2d{ units::degree_t{ botpose[5] } } );

    // See https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#using-wpilib-s-pose-estimator
    timestamp = frc::Timer::GetFPGATimestamp() - units::second_t{ botpose[6]/1000.0 };

    m_poseLogEntry.Append( { april_tag_pose.X().value(),
                            april_tag_pose.Y().value(),
                            april_tag_pose.Rotation().Degrees().value() } );

    return true;
}


// Sets the limelight to new targeting settings
void Limelight::SetPipeline(int pipelineId)
{
    table->PutNumber("pipeline", pipelineId);
}

// Unused
bool Limelight::VisionPose_Unused(frc::Pose2d* AP_Pose)
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
    // frc::SmartDashboard::PutNumber( "tx", targetX );
    // frc::SmartDashboard::PutNumber( "ty", targetY );
    // frc::SmartDashboard::PutNumber( "tv", table->GetNumber("tv", 0.0 ) );
    // if ( camtran.size() > 0 ) {
    //     frc::SmartDashboard::PutNumber( "Yaw", camtran[0]);
    // }
}

// Checks if the limelight sees an AprilTag
bool Limelight::haveValidAprilTag( void ) {
    std::vector<double> bpose{};
    
    bpose = table->GetNumberArray("botpose", defaultValue);
    if ( bpose.size() < 6 ) {
        return false;
    }

    if( table->GetNumber("tv", 0.0 ) < 1.0 ) {
        return false;
    }

    bpose = table->GetNumberArray("camerapose_targetspace", defaultValue);
    if ( bpose.size() >= 6 ) {
            // Seems to give bad data when the April Tag is far away.
        if( std::fabs(bpose[2]) > 3.0 ) return false;
    }

    return true;
}
