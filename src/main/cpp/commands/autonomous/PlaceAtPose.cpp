// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>

#include "commands/autonomous/PlaceAtPose.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/Intake.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceAtPose::PlaceAtPose( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, frc::Pose2d m_targetPose, bool blueSide ) {
  if ( blueSide ) {
    calvin = 1;
  } else {
    calvin = -1;
  }
  AddCommands(
    DriveToPoseCommand( drive, m_targetPose ),
    ArmSet( arm, physical::kPlaceHeight ),
    DriveToPoseCommand( drive, { m_targetPose.X() - ( physical::kPlaceDistance * calvin ), m_targetPose.Y(), m_targetPose.Rotation() } ),
    Intake( grabber, true ),
    frc2::WaitCommand( 0.25_s ),
    DriveToPoseCommand( drive, { m_targetPose.X() + ( physical::kPlaceDistance * calvin ), m_targetPose.Y(), m_targetPose.Rotation() } ),
    ArmSet( arm, -118_deg )
  );
}
