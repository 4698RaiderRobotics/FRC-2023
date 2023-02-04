#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <cmath>

#include "subsystems/Drivetrain.h"

// Balances the robot on the Charge Station
class GyroBalance : public frc2::CommandHelper<frc2::CommandBase, GyroBalance> {
  public:
  
    explicit GyroBalance( Drivetrain* drive );

    void Execute() override;

    bool IsFinished() override;

  private:
    Drivetrain* m_drive;

    frc::ChassisSpeeds speeds;

    units::angle::degree_t errorX;
    units::angle::degree_t errorY;
    units::angle::degree_t currentPitch;
    units::angle::degree_t currentRoll;
    units::meters_per_second_t drivePowerX;
    units::meters_per_second_t drivePowerY;

};
