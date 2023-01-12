#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

// Balances the robot on the Charge Station
class GyroBalance
    : public frc2::CommandHelper<frc2::CommandBase, GyroBalance> {
 public:
  
  explicit GyroBalance( Drivetrain* drive );

  void Execute() override;

 private:
  Drivetrain* m_drive;

  frc::ChassisSpeeds speeds;
};
