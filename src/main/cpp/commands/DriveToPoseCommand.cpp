// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>

#include "commands/DriveToPoseCommand.h"

#include "subsystems/Drivetrain.h"

DriveToPoseCommand::DriveToPoseCommand( Drivetrain *d ) : m_drive{d} {
  AddRequirements( { m_drive } );
}

DriveToPoseCommand::DriveToPoseCommand(  Drivetrain *d, frc::Pose2d targetPose ) 
      : m_drive{d}, m_targetpose{targetPose} {
  AddRequirements( { m_drive } );
}

void DriveToPoseCommand::Initialize() {

  fmt::print( "DriveToPose targetpose = ({:.6}, {:.6}, {:.6}), actual = ({:.6}, {:.6}, {:.6})\n", 
              m_targetpose.X(), m_targetpose.Y(), m_targetpose.Rotation().Degrees(),
              m_drive->GetPose().X(), m_drive->GetPose().Y(), m_drive->GetPose().Rotation().Degrees() );
  fmt::print( "      Distance to Target = X{:.6}, Y{:.6}, A{:.6}\n", 
              m_drive->GetPose().Translation().X() - m_targetpose.Translation().X(),
              m_drive->GetPose().Translation().Y() - m_targetpose.Translation().Y(),
              m_drive->GetPose().Rotation().Degrees() - m_targetpose.Rotation().Degrees() );

  if( m_targetpose.X() > 0.1_cm ) {
      // We already have a target pose since all poses on the field are positive X
      return;
  }

  // Determine which grid target location is the closest.
  frc::Pose2d botpose = m_drive->GetPose();

  if( frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ) {
    m_targetpose = botpose.Nearest( std::span<frc::Pose2d> ( m_drive->redAllianceGridPoints, 9 ) );
  } else {
    m_targetpose = botpose.Nearest( std::span<frc::Pose2d> ( m_drive->blueAllianceGridPoints, 9 ) );
  }
}

void DriveToPoseCommand::Execute() {
    units::degree_t adjustedAngle; 

    m_xSetpoint.position = m_drive->GetPose().X();
    // positive y on the robot is negative x in the target space
    m_ySetpoint.position = m_drive->GetPose().Y();
    // positive omega on the robot is negative in the target space
    m_omegaSetpoint.position = m_drive->GetPose().Rotation().Degrees();

    adjustedAngle = m_targetpose.Rotation().Degrees();
    if ( units::math::abs( m_targetpose.Rotation().Degrees() - m_omegaSetpoint.position ) > 180_deg ) {
      if ( adjustedAngle < 0_deg ) {
        adjustedAngle += 360_deg;
      } else if ( m_omegaSetpoint.position < 0_deg ) {
        m_omegaSetpoint.position += 360_deg;
      }
    }

    m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
        frc::TrapezoidProfile<units::meters>::State {m_targetpose.X()}, m_xSetpoint };
    m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, 
        frc::TrapezoidProfile<units::meters>::State {m_targetpose.Y()}, m_ySetpoint };
    m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, 
        frc::TrapezoidProfile<units::degrees>::State {adjustedAngle}, m_omegaSetpoint };

    m_xSetpoint = m_xProfile.Calculate( 20_ms );
    m_ySetpoint = m_yProfile.Calculate( 20_ms );
    m_omegaSetpoint = m_omegaProfile.Calculate( 20_ms );
    if ( m_omegaSetpoint.position > 180_deg ) {
      m_omegaSetpoint.position -= 360_deg;
    }

    frc::ChassisSpeeds speeds;
    speeds.vx = m_xSetpoint.velocity;
    speeds.vy = m_ySetpoint.velocity;
    speeds.omega = m_omegaSetpoint.velocity;

  //  fmt::print( "DriveToPose targetpose = ({:.6}, {:.6}, {:.6}), actual = ({:.6}, {:.6}, {:.6})\n", 
  //             m_targetpose.X(), m_targetpose.Y(), m_targetpose.Rotation().Degrees(),
  //             m_drive->GetPose().X(), m_drive->GetPose().Y(), m_drive->GetPose().Rotation().Degrees() );
  //  fmt::print( "DriveToPose theta = setpt({:.6}), actual{:.6}, omega({:.6})\n", m_omegaSetpoint.position, 
  //             m_drive->GetPose().Rotation().Degrees(), m_omegaSetpoint.velocity );
  //  fmt::print( "DriveToPose Distance to Target = L{:.6}, A{:.6}\n", 
  //               m_targetpose.Translation().Distance( m_drive->GetPose().Translation() ),
  //               m_drive->GetPose().Rotation().Degrees() - m_targetpose.Rotation().Degrees() );

    m_drive->Drive( speeds );
}

bool DriveToPoseCommand::IsFinished() {
  units::meter_t distance_error;
  units::degree_t angle_error;

  distance_error = m_targetpose.Translation().Distance( m_drive->GetPose().Translation() );
  angle_error = units::math::abs( m_targetpose.Rotation().Degrees() - m_drive->GetPose().Rotation().Degrees() );
  if( angle_error > 180_deg ) angle_error = 360_deg - angle_error;

    // fmt::print( "DriveToPoseCommand::IsFinished target( {}, {}, {} ), robot( {}, {}, {} ) = L{}, A{}\n", 
    //              m_targetpose.X(), m_targetpose.Y(), m_targetpose.Rotation().Degrees(), 
    //              m_drive->GetPose().X(), m_drive->GetPose().Y(), m_drive->GetPose().Rotation().Degrees(), 
    //              distance_error, angle_error );

  bool atTargetLocation = distance_error < 1.0_cm && angle_error < 0.15_deg;
  if( atTargetLocation ) { 
    fmt::print( "DriveToPoseCommand::IsFinished target( {}, {}, {} ), robot( {}, {}, {} ) = L{}, A{}\n", 
                 m_targetpose.X(), m_targetpose.Y(), m_targetpose.Rotation().Degrees(), 
                 m_drive->GetPose().X(), m_drive->GetPose().Y(), m_drive->GetPose().Rotation().Degrees(), 
                 distance_error, angle_error );
      // Make sure the drive is stopped.
    //m_drive->ArcadeDrive( 0.0, 0.0, 0.0 );
    m_drive->StopDrive( );
  }

  return atTargetLocation;
}
