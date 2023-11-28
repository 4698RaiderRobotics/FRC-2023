// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/FollowTrajectory.h"

FollowTrajectory::FollowTrajectory( Drivetrain *drive, frc::Pose2d endPose )
          : m_drive{ drive }, m_endPose{ endPose } {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements( { drive } );
  hasTraj = false;
}

FollowTrajectory::FollowTrajectory(Drivetrain *drive, frc::Trajectory trajectory, frc::Rotation2d endAngle, bool invert)
          : m_drive{drive}, m_trajectory{trajectory}, m_finishHeading{endAngle}, m_invert{invert} {
  AddRequirements( { drive } );
  m_endPose = trajectory.Sample(trajectory.TotalTime()).pose;
  hasTraj = true;
}

// Called when the command is initially scheduled.
void FollowTrajectory::Initialize() {
  fmt::print("FollowTrajectory::Initialize()\n");
  if (!hasTraj) {
    m_startPose = m_drive->GetPose();
    m_startHeading = m_startPose.Rotation();
    m_finishHeading = m_endPose.Rotation();
    m_relativePose = m_endPose.RelativeTo(m_startPose);

    auto linear_tangent =  frc::Rotation2d{ m_endPose.X().value() - m_startPose.X().value(), m_endPose.Y().value() - m_startPose.Y().value()};

    frc::Pose2d m_midPose = {(m_endPose.X() + m_startPose.X()) / 2.0, (m_endPose.Y() + m_startPose.Y()) / 2.0, linear_tangent.Degrees()};

    fmt::print( "   Start Pose: {}, {}, Rotation {}\n", m_startPose.X(), m_startPose.Y(), linear_tangent.Degrees() );
    fmt::print( "   End Pose {}, {}\n", m_endPose.X(), m_endPose.Y() );

  
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    {m_startPose.X(), m_startPose.Y(), linear_tangent}, 
    m_midPose,
    {m_endPose.X(), m_endPose.Y(), linear_tangent}},
      m_config );

    fmt::print("Trajectory time: {}\n", m_trajectory.TotalTime());

  }

  m_drive->m_field.GetObject("trajectory")->SetTrajectory(m_trajectory);
  if(m_invert) {
    m_autoElapsed = m_trajectory.TotalTime();
    m_endPose = m_trajectory.InitialPose();
  } else {
    m_autoElapsed = 0_ms;
  }
  fmt::print("End Pose: {}, {}, {}\n", m_endPose.X(), m_endPose.Y(), m_endPose.Rotation().Degrees());
//  frc::Trajectory::State start = m_trajectory.Sample( 0_s );
//  m_drive->ResetPose( start.pose );
}

// Called repeatedly when this Command is scheduled to run
void FollowTrajectory::Execute() {
  auto goal = m_trajectory.Sample( m_autoElapsed );

  
  frc::SmartDashboard::PutNumber("Goal X", goal.pose.X().value());
  frc::SmartDashboard::PutNumber("Goal Y", goal.pose.Y().value());
  frc::SmartDashboard::PutNumber("End Angle", m_finishHeading.Degrees().value());

  m_drive->DriveTrajectory( goal, m_finishHeading );

  if(m_invert) {
    m_autoElapsed -= 20_ms;
  } else {
    m_autoElapsed += 20_ms;
  }
}

// Returns true when the command should end.
bool FollowTrajectory::IsFinished() {
  if(m_invert) {
    atTarget = m_autoElapsed < 0_s && units::math::abs(m_endPose.Rotation().Degrees() - m_drive->GetPose().Rotation().Degrees()) < 1_deg;
  } else {
    atTarget = m_trajectory.TotalTime() < m_autoElapsed - 100_ms && units::math::abs(m_finishHeading.Degrees() - m_drive->GetPose().Rotation().Degrees()) < 1_deg;
  }
  if (atTarget) {
    fmt::print("FollowTrajectory::IsFinished()\n");
    m_drive->StopDrive();
  }
  return atTarget;
}
