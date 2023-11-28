// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceHigh.h"

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/ArmSet.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceHigh::PlaceHigh(Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, LEDs *leds) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    ArmSet(arm, physical::kArmUpperPlaceHeight, physical::kWristUpperPlaceHeight, 0.3),

    frc2::WaitCommand(0.2_s),

    frc2::InstantCommand([this, grabber] { grabber->HandleCube(); }, { grabber, leds }),
    
    ArmSet(arm, -118_deg, 50_deg, 0.3)
  );
}
