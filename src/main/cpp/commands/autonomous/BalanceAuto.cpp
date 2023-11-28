// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>

#include "commands/autonomous/BalanceAuto.h"
#include "commands/autonomous/PlaceAtPose.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/TestProfileMove.h"
#include "commands/GyroBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
BalanceAuto::BalanceAuto(Drivetrain* drive, ArmSubsystem* arm, GrabberSubsystem* grabber, LEDs *leds) {
  // frc::Pose2d redAllianceTargetPoints[2] = { drive->redAllianceGridPoints[3], drive->redAllianceGridPoints[5] };
  // frc::Pose2d blueAllianceTargetPoints[2] = { drive->blueAllianceGridPoints[3], drive->blueAllianceGridPoints[5] };
  fmt::print("BalanceAuto::BalanceAuto\n");

  units::meter_t auto_drive_distance = 2.25_m;
  // If on blue side, do blue auto
  if (drive->GetPose().X() < 7.5_m) {
    //m_targetpose = drive->GetPose().Nearest(std::span<frc::Pose2d>(blueAllianceTargetPoints, 2));
    m_targetpose = drive->blueAllianceGridPoints[4];
    AddCommands(
      PlaceAtPose(drive, arm, grabber, leds, m_targetpose, true),
      //      DriveToPoseCommand( drive, { m_targetpose.X() + auto_drive_distance, m_targetpose.Y(), 180_deg } ),
      TestProfileMove(drive, -auto_drive_distance, TestProfileMove::FORWARD),
      GyroBalance(drive)
    );
    // If on red side, do red auto
  }
  else if (drive->GetPose().X() > 7.5_m) {
    //m_targetpose = drive->GetPose().Nearest(std::span<frc::Pose2d>(redAllianceTargetPoints, 2));
    m_targetpose = drive->redAllianceGridPoints[4];
    AddCommands(
      PlaceAtPose(drive, arm, grabber, leds, m_targetpose, false),
      //      DriveToPoseCommand( drive, { m_targetpose.X() - auto_drive_distance, m_targetpose.Y(), 0_deg } ),
      TestProfileMove(drive, -auto_drive_distance, TestProfileMove::FORWARD),
      GyroBalance(drive)
    );
  }
}
