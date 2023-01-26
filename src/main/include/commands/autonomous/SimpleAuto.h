#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include "subsystems/Drivetrain.h"
#include "commands/autonomous/FollowTrajectory.h"


class SimpleAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, SimpleAuto> {
    public:
        SimpleAuto( Drivetrain *drive );

    private:
        frc::Trajectory m_simpleAutoTrajectory;
};