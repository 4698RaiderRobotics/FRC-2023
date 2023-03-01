// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>

#include "commands/DriveToPoseCommand.h"

#include "subsystems/Drivetrain.h"

DriveToPoseCommand::DriveToPoseCommand( Drivetrain *d ) : m_drive{d} {
  AddRequirements( { m_drive } );

  for( int n=0; n<9; ++n ) {
    redAllianceGridPoints[n] = { 14.1_m, 20_in + 22_in*n, 0_deg };
    blueAllianceGridPoints[n] = { 1.95_m, 20_in + 22_in*n, 180_deg };
  }
}

// Called when the command is initially scheduled.
void DriveToPoseCommand::Initialize() {
  // Determine which grid target location is the closest.
  frc::Pose2d botpose = m_drive->GetPose();

  if( frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ) {
    m_targetpose = botpose.Nearest( std::span<frc::Pose2d> ( redAllianceGridPoints, 9 ) );
  } else {
    m_targetpose = botpose.Nearest( std::span<frc::Pose2d> ( blueAllianceGridPoints, 9 ) );
  }
/*
  if( frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ) {
      // We are the Red Alliance
    if( botpose.Y() < 33.2_in ) {
        // Target the Cone Grid Right of AT#1
      m_targetpose = frc::Pose2d{ 14.1_m, 20.13_in, 0_deg };
    } else if( botpose.Y() > 33.2_in &&  botpose.Y() < 33.2_in + 18.25_in ) {
        // Target the Cube Grid at AT#1
      m_targetpose = frc::Pose2d{ 14.1_m, 42.13_in, 0_deg };
    } else if( botpose.Y() > 33.2_in + 18.25_in &&  botpose.Y() < 33.2_in + 18.25_in + 47.75_in/2 ) {
        // Target the Cone Grid Left of AT#1
      m_targetpose = frc::Pose2d{ 14.1_m, 64.13_in, 0_deg };
    }
  } else {
      // We are the Blue Alliance

  }
*/
}

// Called repeatedly when this Command is scheduled to run
void DriveToPoseCommand::Execute() {
    m_xSetpoint.position = m_drive->GetPose().X();
    // positive y on the robot is negative x in the target space
    m_ySetpoint.position = m_drive->GetPose().Y();
    // positive omega on the robot is negative in the target space
    m_omegaSetpoint.position = m_drive->GetPose().Rotation().Degrees();

    m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
        frc::TrapezoidProfile<units::meters>::State {m_targetpose.X()}, m_xSetpoint };
    m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
        frc::TrapezoidProfile<units::meters>::State {m_targetpose.Y()}, m_ySetpoint };
    m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, 
        frc::TrapezoidProfile<units::degrees>::State {m_targetpose.Rotation().Degrees()}, m_omegaSetpoint };

    m_xSetpoint = m_xProfile.Calculate( 20_ms );
    m_ySetpoint = m_yProfile.Calculate( 20_ms );
    m_omegaSetpoint = m_omegaProfile.Calculate( 20_ms );

    frc::ChassisSpeeds speeds;
    speeds.vx = m_xSetpoint.velocity;
    speeds.vy = m_ySetpoint.velocity;
    speeds.omega = m_omegaSetpoint.velocity;

//    fmt::print( "DriveToPose theta = setpt({}), actual{}, omega({})\n", m_omegaSetpoint.position, 
//               m_drive->GetPose().Rotation().Degrees(), m_omegaSetpoint.velocity );
//    fmt::print( "DriveToPose Distance to Target = L{}, A{}\n", 
//                 m_targetpose.Translation().Distance( m_drive->GetPose().Translation() ),
//                 m_drive->GetPose().Rotation().Degrees() - m_targetpose.Rotation().Degrees() );

    m_drive->Drive( speeds, false );
}

// Returns true when the command should end.
bool DriveToPoseCommand::IsFinished() {
  units::meter_t distance_error;
  units::degree_t angle_error;

  distance_error = m_targetpose.Translation().Distance( m_drive->GetPose().Translation() );
  angle_error = units::math::abs( m_drive->GetPose().Rotation().Degrees() - m_targetpose.Rotation().Degrees() );

  bool atTargetLocation = distance_error < 1.5_cm && angle_error < 0.25_deg;
  if( atTargetLocation ) { 
    fmt::print( "DriveToPoseCommand::IsFinished target( {}, {}, {} ), robot( {}, {}, {} ) = L{}, A{}\n", 
                 m_targetpose.X(), m_targetpose.Y(), m_targetpose.Rotation().Degrees(), 
                 m_drive->GetPose().X(), m_drive->GetPose().Y(), m_drive->GetPose().Rotation().Degrees(), 
                 distance_error, angle_error );
  }

  return atTargetLocation;
}

// Called once the command ends or is interrupted.
void DriveToPoseCommand::End(bool interrupted) {
  m_drive->Drive( 0.0, 0.0, 0.0 );
}
