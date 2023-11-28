// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/TrajPracticeAuto.h"

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include "commands/autonomous/FollowTrajectory.h"
#include "commands/autonomous/PlaceAtPose.h"
#include "commands/PlaceHigh.h"
#include "commands/ArmSet.h"
#include "commands/TestProfileMove.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TrajPracticeAuto::TrajPracticeAuto(Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, LEDs *leds) {

  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::Pose2d m_targetPose = drive->blueAllianceGridPoints[4];
  AddCommands(
    PlaceAtPose(drive, arm, grabber, leds, m_targetPose, true),

    frc2::ParallelCommandGroup(FollowTrajectory(drive, drive->twoPiecePrac, 90_deg),
                               ArmSet(arm, -90_deg, 12_deg, 0.3)),
    
    frc2::ParallelCommandGroup(TestProfileMove(drive, 0.5_m, TestProfileMove::FORWARD),
                               frc2::InstantCommand([this, grabber] { grabber->HandleCube(); }, { grabber, leds })),
    
    ArmSet(arm, -118_deg, 50_deg, 0.3),
    
    FollowTrajectory(drive, drive->twoPiecePrac2, 180_deg),
    
    PlaceHigh(drive, arm, grabber, leds)
  );
}
