// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceGamePiece.h"
#include "Config.h"

#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/TestProfileMove.h"
#include "commands/OpenGrabber.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceGamePiece::PlaceGamePiece( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, 
                                units::degree_t angle ) {
  AddCommands(
    DriveToPoseCommand{ drive },
    ArmSet( arm, angle ),
    TestProfileMove( drive, physical::kPlaceDistance, TestProfileMove::FORWARD ),
    #if defined(Claw)
    OpenGrabber( grabber ),
    #endif
    TestProfileMove( drive, -physical::kPlaceDistance, TestProfileMove::FORWARD ),
    ArmSet( arm, -90_deg )
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