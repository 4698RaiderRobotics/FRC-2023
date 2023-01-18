#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Limelight.h"
#include "subsystems/Drivetrain.h"

class TargetLimelight
    : public frc2::CommandHelper<frc2::CommandBase, TargetLimelight> {
 public:
  
  explicit TargetLimelight( Drivetrain* drive, Limelight* limelight );

  void Execute() override;

 private:
  Drivetrain* m_drive;
  Limelight* m_limelight;
  
};
