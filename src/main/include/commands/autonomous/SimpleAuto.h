#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

class SimpleAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, SimpleAuto> {
  public:
    SimpleAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, 
                units::degree_t angle );

  private:
    frc::Trajectory m_simpleAutoTrajectory;
    frc::Trajectory m_trajectory;
};