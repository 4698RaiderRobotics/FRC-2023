// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceGamePiece.h"

#include "commands/TargetLimelight.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/TestProfileMove.h"
#include "commands/OpenGrabber.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceGamePiece::PlaceGamePiece( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, Limelight *limelight, 
                                frc::Pose2d targetPose, units::degree_t angle ) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
//    TargetLimelight{ drive, limelight, targetPose },
    DriveToPoseCommand{ drive },
    ArmSet( angle, arm ),
    TestProfileMove( 18_in, TestProfileMove::FORWARD, drive ),
    OpenGrabber( grabber ),
    TestProfileMove( -18_in, TestProfileMove::FORWARD, drive ),
    ArmSet( -90_deg, arm )
);
  m_timer.Restart();
}

frc2::Command::InterruptionBehavior PlaceGamePiece::GetInterruptionBehavior() const {
  if ( m_timer.HasElapsed( 0.5_s ) ) {
    return frc2::Command::InterruptionBehavior::kCancelSelf;
  } else {
    return frc2::Command::InterruptionBehavior::kCancelIncoming;
  }
}