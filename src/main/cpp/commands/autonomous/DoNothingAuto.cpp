// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/DoNothingAuto.h"
#include "commands/autonomous/FollowTrajectory.h"
#include "commands/autonomous/PlaceAtPose.h"
#include "commands/ArmSet.h"
#include "commands/PlaceHigh.h"
#include "commands/TestProfileMove.h"

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
DoNothingAuto::DoNothingAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, LEDs *leds ) {
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "paths" / "Traj_Practice.wpilib.json";
  m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::Pose2d m_targetPose = drive->blueAllianceGridPoints[4];
  AddCommands(
    // frc2::WaitCommand(0.5_s), 
    // frc2::InstantCommand([this, arm] {arm->ArmOn(-0.75, 0.5); }, { arm }),
    // frc2::WaitCommand(0.4_s), 
    // frc2::InstantCommand([this, arm] {arm->ArmOn(0.0, 0.0); }, { arm })

    PlaceAtPose(drive, arm, grabber, leds, m_targetPose, true),

    frc2::ParallelCommandGroup(FollowTrajectory(drive, {m_targetPose.X() - (physical::kPlaceDistance), 
                                                        m_targetPose.Y() + 1_m, m_targetPose.Rotation().Degrees() - 90_deg}),
                               ArmSet(arm, -90_deg, 12_deg, 0.3)),
    
    frc2::ParallelCommandGroup(TestProfileMove(drive, 0.5_m, TestProfileMove::FORWARD),
                               frc2::InstantCommand([this, grabber] { grabber->HandleCube(); }, { grabber, leds })),
    
    ArmSet(arm, -118_deg, 50_deg, 0.3),
    
    FollowTrajectory(drive, {m_targetPose.X() - (physical::kPlaceDistance), 
                            m_targetPose.Y(), m_targetPose.Rotation()}),
    
    PlaceHigh(drive, arm, grabber, leds)
  );
}