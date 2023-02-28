// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>

#include "commands/DriveToPoseCommand.h"

#include "subsystems/Drivetrain.h"

DriveToPoseCommand::DriveToPoseCommand( Drivetrain *d ) : m_drive{d} {
   AddRequirements( { m_drive } );
}

// Called when the command is initially scheduled.
void DriveToPoseCommand::Initialize() {
  // Determine which grid is the closest.
  frc::Pose2d botpose = m_drive->GetPose();

  if( frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ) {
      // We are the Red Alliance
    if( botpose.Y() < 33.2_in ) {
        // Target the Cone Grid Right of AT#1
      m_targetpose = frc::Pose2d{ 580_in, 20.13_in, 0_deg };
    } else if( botpose.Y() > 33.2_in &&  botpose.Y() < 33.2_in + 18.25_in ) {
        // Target the Cube Grid at AT#1
      m_targetpose = frc::Pose2d{ 580_in, 42.13_in, 0_deg };
    } else if( botpose.Y() > 33.2_in + 18.25_in &&  botpose.Y() < 33.2_in + 18.25_in + 47.75_in/2 ) {
        // Target the Cone Grid Left of AT#1
      m_targetpose = frc::Pose2d{ 580_in, 64.13_in, 0_deg };
    }
  } else {
      // We are the Blue Alliance

  }

  m_drive->GetPose();
  m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
      frc::TrapezoidProfile<units::meters>::State {m_targetpose.X()}, {botpose.X()} };
  m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
      frc::TrapezoidProfile<units::meters>::State {m_targetpose.Y()}, {botpose.Y()} };
  m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, 
      frc::TrapezoidProfile<units::degrees>::State {m_targetpose.Rotation().Degrees()}, {botpose.Rotation().Degrees()} };

    m_profile_time = 0_ms;
}

// Called repeatedly when this Command is scheduled to run
void DriveToPoseCommand::Execute() {
    m_profile_time += 20_ms;

    frc::ChassisSpeeds speeds;

    speeds.vx = m_xProfile.Calculate( m_profile_time ).velocity;
    speeds.vy = m_yProfile.Calculate( m_profile_time ).velocity;
    speeds.omega = m_omegaProfile.Calculate( m_profile_time ).velocity;

    m_drive->Drive( speeds, false );
}

// Returns true when the command should end.
bool DriveToPoseCommand::IsFinished() {
  return m_xProfile.IsFinished( m_profile_time ) && m_yProfile.IsFinished( m_profile_time ) && m_omegaProfile.IsFinished( m_profile_time );
}
