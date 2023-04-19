// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Config.h"

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>


#include <units/current.h>
#include "Constants.h"
class ArmSubsystem;
class GrabberSubsystem : public frc2::SubsystemBase {
 public:
  GrabberSubsystem( frc::PowerDistribution &, ArmSubsystem* );

  void Periodic() override;

  units::ampere_t GetCurrent();
  void GrabberTest();
  void Spin( double speed );

  void Cone( bool direction );

  void Cube( bool direction );

  void HandleCube( void );
  void HandleCone( void );
 private:
  double m_spin_speed = 1.0;
  double m_cone_intake_speed = 0.5;
  double m_cone_shoot_speed = 1.0;
  double m_cube_intake_speed = 0.5;
  double m_cube_shoot_fast_speed = 0.75;
  double m_cube_shoot_slow_speed = 0.25;

  double m_cone_max_amps = 20.0;
  double m_cube_max_amps = 10.0;
  double m_target_amps = 5.0;
  frc::PowerDistribution &m_pdp;
  ArmSubsystem *m_arm;
  rev::CANSparkMax m_intake{14, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  units::second_t m_startTime;
  bool m_isEjecting;
  bool m_loadingCone{false}, m_hasCone{ false };
  bool m_loadingCube{false}, m_hasCube{ false };
};
