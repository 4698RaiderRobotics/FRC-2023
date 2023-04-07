// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Config.h"

#include <frc2/command/WaitCommand.h>

#include "commands/autonomous/LeaveAuto.h"
#include "commands/autonomous/PlaceAtPose.h"
#include "commands/DriveToPoseCommand.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
LeaveAuto::LeaveAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber ) {
  frc::Pose2d redAllianceTargetPoints[2] = { drive->redAllianceGridPoints[0], drive->redAllianceGridPoints[8] };
  frc::Pose2d blueAllianceTargetPoints[2] = { drive->blueAllianceGridPoints[0], drive->blueAllianceGridPoints[8] };

  units::meter_t drive_out_distance = 4.0_m;
  
  if ( drive->GetPose().X() < 7.5_m ) {
    m_targetpose = drive->GetPose().Nearest( std::span<frc::Pose2d> ( blueAllianceTargetPoints, 2 ) );
    AddCommands(
      PlaceAtPose( drive, arm, grabber, m_targetpose, true ),
      DriveToPoseCommand( drive, { m_targetpose.X() + drive_out_distance, m_targetpose.Y(), 180_deg } )
    );
  // If on red side, do red auto
  } else if ( drive->GetPose().X() > 7.5_m ) {
    m_targetpose = drive->GetPose().Nearest( std::span<frc::Pose2d> ( redAllianceTargetPoints, 2 ) );
    AddCommands(
      PlaceAtPose( drive, arm, grabber, m_targetpose, false ),
      DriveToPoseCommand( drive, { m_targetpose.X() - drive_out_distance, m_targetpose.Y(), 0_deg } )
    );
  }
}
