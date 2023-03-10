
#include "commands/TargetLimelight.h"

TargetLimelight::TargetLimelight( Drivetrain* drive, Limelight* limelight, frc::Pose2d targetPose )
    : m_drive{ drive }, m_limelight{ limelight }, m_targetPose{ targetPose } {
        AddRequirements( { drive, limelight } );
    }

void TargetLimelight::Initialize() {
    m_trajFinished = false;
    m_poseFound = m_limelight->TargetRobot_AT_Unused( m_pose );
    if( !m_poseFound ) {
        m_trajFinished = true;
        return;
    }
/*
    frc::SmartDashboard::PutNumber( "Current X Pose", m_pose.X().value() );
    frc::SmartDashboard::PutNumber( "Current Y Pose", -m_pose.Y().value() );
    frc::SmartDashboard::PutNumber( "Current Omega Pose", -m_pose.Rotation().Degrees().value() );
    // positive x on the robot is positive y in the target space
    m_xSetpoint = { m_pose.X() };
    // positive y on the robot is negative x in the target space
    m_ySetpoint = { -m_pose.Y() };
    // positive omega on the robot is negative in the target space
    m_omegaSetpoint = { -m_pose.Rotation().Degrees() };
*/
    // target pose X-coord
    m_xGoal = { m_targetPose.X() };
    // target pose Y-Coord
    m_yGoal = { -m_targetPose.Y() };
    // target rotation 
    m_omegaGoal = { -m_targetPose.Rotation().Degrees() };

    m_profile_time = 0_ms;
}

void TargetLimelight::Execute() {
    if( m_trajFinished ) return;

    m_poseFound = m_limelight->TargetRobot_AT_Unused( m_pose );
    frc::SmartDashboard::PutNumber( "Current X Pose", m_pose.X().value() );
    frc::SmartDashboard::PutNumber( "Current Y Pose", -m_pose.Y().value() );
    frc::SmartDashboard::PutNumber( "Current Omega Pose", -m_pose.Rotation().Degrees().value() );
    m_drive->PoseToNetworkTables();

    if ( m_poseFound ) {
        m_profile_time = 0_ms;
        m_xSetpoint.position = m_pose.X();
        // positive y on the robot is negative x in the target space
        m_ySetpoint.position = -m_pose.Y();
        // positive omega on the robot is negative in the target space
        m_omegaSetpoint.position = -m_pose.Rotation().Degrees();
        m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, m_xGoal, m_xSetpoint };
        m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, m_yGoal, m_ySetpoint };
        m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, m_omegaGoal, m_omegaSetpoint };
    }

    m_profile_time += 20_ms;
    m_xSetpoint = m_xProfile.Calculate( m_profile_time );
    m_ySetpoint = m_yProfile.Calculate( m_profile_time );
    m_omegaSetpoint = m_omegaProfile.Calculate( m_profile_time );

    m_drive->Drive( frc::ChassisSpeeds{ m_xSetpoint.velocity, m_ySetpoint.velocity, m_omegaSetpoint.velocity }, false );

    if ( m_xProfile.IsFinished( m_profile_time ) && m_yProfile.IsFinished( m_profile_time ) && m_omegaProfile.IsFinished( m_profile_time ) ) {
        m_trajFinished = true;
    }
}

bool TargetLimelight::IsFinished() {
    return m_trajFinished;
}