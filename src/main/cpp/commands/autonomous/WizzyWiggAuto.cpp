// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/WizzyWiggAuto.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/TestProfileMove.h"
#include "commands/OpenGrabber.h"
#include "commands/autonomous/FollowTrajectory.h"
#include "commands/TestProfileMove.h"
#include "commands/TurnToAngle.h"
#include "commands/GyroBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
WizzyWiggAuto::WizzyWiggAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, units::degree_t angle ) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    DriveToPoseCommand( drive, drive->redAllianceGridPoints[8] ),
    ArmSet( angle, arm ),
    TestProfileMove( 17_in, TestProfileMove::FORWARD, drive ),
    OpenGrabber( grabber ),
    TestProfileMove( -17_in, TestProfileMove::FORWARD, drive ),
    ArmSet( -90_deg, arm ),
    DriveToPoseCommand( drive, drive->redAllianceGridPoints[5] ),
    DriveToPoseCommand( drive, { drive->redAllianceGridPoints[5].X() - 1.5_m, drive->redAllianceGridPoints[5].Y(), 0_deg } ),
    GyroBalance( drive )
  );
}
