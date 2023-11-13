// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

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
    frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(frc2::WaitCommand(0.5_s), frc2::InstantCommand([this, arm] {arm->ArmOn(-0.75, 0.5); }, { arm }),
      frc2::WaitCommand(0.4_s), frc2::InstantCommand([this, arm] {arm->ArmOn(0.0, 0.0); }, { arm })), 
      DriveToPoseCommand(drive, { m_targetPose.X() - (physical::kPlaceDistance * alliance), m_targetPose.Y(), m_targetPose.Rotation() })),
    // frc2::WaitCommand(0.5_s),
    // frc2::InstantCommand([this, arm] {arm->ArmOn(-0.75, 0.5); }, { arm }),
    // frc2::WaitCommand(0.4_s),
    // frc2::InstantCommand([this, arm] {arm->ArmOn(0.0, 0.0); }, { arm }),
    //ArmSet(arm, arm->GetArmAngle() - 10_deg, arm->GetWristAngle() + 10_deg, 0.3, true ),
    DriveToPoseCommand( drive, { m_targetPose.X() - ( physical::kPlaceDistance * alliance ), m_targetPose.Y(), m_targetPose.Rotation() } ),
    ArmSet(arm, physical::kArmUpperPlaceHeight, physical::kWristUpperPlaceHeight, 0.3),
    frc2::InstantCommand( [this, grabber] { grabber->Spin( -0.35 ) ;}, { grabber } ),
    frc2::WaitCommand( 0.25_s ),
    frc2::InstantCommand( [this, grabber] { grabber->Spin( 0.0 ) ;}, { grabber } ),
    ArmSet(arm, -118_deg, 50_deg, 0.3)
  );
}
