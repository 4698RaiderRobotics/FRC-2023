#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_acceleration.h>

#include "subsystems/Limelight.h"
#include "subsystems/Drivetrain.h"

class TargetLimelight
    : public frc2::CommandHelper<frc2::CommandBase, TargetLimelight> {
 public:
  
  explicit TargetLimelight( Drivetrain* drive, Limelight* limelight, frc::Pose2d targetPose );

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
  Drivetrain* m_drive;
  Limelight* m_limelight;

  frc::Pose2d m_pose;
  frc::Pose2d m_targetPose;

  bool m_poseFound;
  bool m_trajFinished = false;
  int poseIteration = 0;
  units::second_t m_profile_time;

  frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{ 3_mps, 0.5_mps_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_omegaConstraints{ 90_deg_per_s, 10_deg_per_s_sq };
  frc::TrapezoidProfile<units::meters>::State m_xGoal;
  frc::TrapezoidProfile<units::meters>::State m_yGoal;
  frc::TrapezoidProfile<units::degrees>::State m_omegaGoal;
  frc::TrapezoidProfile<units::meters>::State m_xSetpoint;
  frc::TrapezoidProfile<units::meters>::State m_ySetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_omegaSetpoint;
  
  frc::TrapezoidProfile<units::meters> m_xProfile{ m_linearConstraints, m_xGoal, m_xSetpoint };
  frc::TrapezoidProfile<units::meters> m_yProfile{ m_linearConstraints, m_yGoal, m_ySetpoint };
  frc::TrapezoidProfile<units::degrees> m_omegaProfile{ m_omegaConstraints, m_omegaGoal, m_omegaSetpoint };
};
