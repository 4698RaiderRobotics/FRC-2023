// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/autonomous/PlaceAtPose.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/Intake.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceAtPose::PlaceAtPose( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, frc::Pose2d m_targetPose, bool blueSide ) {
  alliance = blueSide ? 1 : -1;
  AddCommands(
    DriveToPoseCommand( drive, m_targetPose ),
    ArmSet(arm, physical::kArmMidPlaceHeight, physical::kWristMidPlaceHeight, 0.5),
    DriveToPoseCommand( drive, { m_targetPose.X() - ( physical::kPlaceDistance * alliance ), m_targetPose.Y(), m_targetPose.Rotation() } ),
    frc2::InstantCommand( [this, grabber] { grabber->Spin( -0.5 ) ;}, { grabber } ),
    frc2::WaitCommand( 0.25_s ),
    frc2::InstantCommand( [this, grabber] { grabber->Spin( 0.0 ) ;}, { grabber } ),
    DriveToPoseCommand( drive, { m_targetPose.X() + ( physical::kPlaceDistance * alliance ), m_targetPose.Y(), m_targetPose.Rotation() } ),
    ArmSet(arm, -118_deg, 45_deg, 0.5)
  );
}
