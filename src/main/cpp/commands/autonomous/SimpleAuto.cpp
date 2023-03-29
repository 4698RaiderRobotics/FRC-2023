// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>

#include "commands/autonomous/SimpleAuto.h"
#include "commands/autonomous/PlaceAtPose.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SimpleAuto::SimpleAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber ) {
  frc::Pose2d redAllianceTargetPoints[2] = { drive->redAllianceGridPoints[3], drive->redAllianceGridPoints[5] };
  frc::Pose2d blueAllianceTargetPoints[2] = { drive->blueAllianceGridPoints[3], drive->blueAllianceGridPoints[5] };
  fmt::print( "WizzyWiggAuto::WizzyWiggAuto\n" );
  // If on blue side, do blue auto
  if ( drive->GetPose().X() < 7.5_m ) {
    fmt::print( "Blue Alliance\n");
    m_targetpose = drive->GetPose().Nearest( std::span<frc::Pose2d> ( blueAllianceTargetPoints, 2 ) );
    AddCommands(
      PlaceAtPose( drive, arm, grabber, m_targetpose, true )
    );
  // If on red side, do red auto
  } else if ( drive->GetPose().X() > 7.5_m ) {
    fmt::print( "Red Alliance\n");
    m_targetpose = drive->GetPose().Nearest( std::span<frc::Pose2d> ( redAllianceTargetPoints, 2 ) );
    AddCommands(
      PlaceAtPose( drive, arm, grabber, m_targetpose, false )
    );
  }
}
