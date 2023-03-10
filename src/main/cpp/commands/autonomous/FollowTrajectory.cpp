// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/FollowTrajectory.h"

FollowTrajectory::FollowTrajectory( Drivetrain *drive, frc::Trajectory trajectory, 
                                    frc::Rotation2d startHeading, frc::Rotation2d finishHeading )
          : m_drive{ drive }, m_trajectory{ trajectory }, m_startHeading{startHeading}, m_finishHeading{finishHeading} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements( { drive } );
}

// Called when the command is initially scheduled.
void FollowTrajectory::Initialize() {
  m_drive->m_field.GetObject("trajectory")->SetTrajectory(m_trajectory);
  m_autoElapsed = 0_ms;
//  frc::Trajectory::State start = m_trajectory.Sample( 0_s );
//  m_drive->ResetPose( start.pose );
}

// Called repeatedly when this Command is scheduled to run
void FollowTrajectory::Execute() {
  double percent_done = m_autoElapsed / m_trajectory.TotalTime();
  auto goal = m_trajectory.Sample( m_autoElapsed );
  m_drive->DriveTrajectory( goal, (m_finishHeading - m_startHeading) * percent_done  + m_startHeading );
  m_autoElapsed += 20_ms;
}

// Returns true when the command should end.
bool FollowTrajectory::IsFinished() {
  return m_trajectory.TotalTime() < m_autoElapsed;
}
