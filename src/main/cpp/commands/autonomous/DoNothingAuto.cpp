// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/DoNothingAuto.h"

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
DoNothingAuto::DoNothingAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber ) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    frc2::WaitCommand(0.5_s), 
    frc2::InstantCommand([this, arm] {arm->ArmOn(-0.75, 0.5); }, { arm }),
    frc2::WaitCommand(0.4_s), 
    frc2::InstantCommand([this, arm] {arm->ArmOn(0.0, 0.0); }, { arm })
  );
}
