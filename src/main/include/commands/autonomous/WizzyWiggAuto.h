// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

class WizzyWiggAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 WizzyWiggAuto> {
 public:
  WizzyWiggAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, units::degree_t angle );


  
};
