// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TestProfileMove.h"

#include <units/acceleration.h>
#include <units/velocity.h>

#include "subsystems/Drivetrain.h"

TestProfileMove::TestProfileMove( Drivetrain *drive, units::meter_t distance, Direction d )
    :  m_drive{ drive }, m_distance{ distance }, m_direction{ d } {
  AddRequirements( { drive } );

}

void TestProfileMove::Initialize( void ) {
  m_Setpoint.position = 0_m;
  m_Setpoint.velocity = 0_mps;
  m_Goal.position = m_distance;
  m_Goal.velocity = 0.0_mps;
  m_profile = frc::TrapezoidProfile<units::meters>{ m_linearConstraints, m_Goal, m_Setpoint };
  m_elapsed_time = 0_ms;
}

void TestProfileMove::Execute( void ) {
  m_elapsed_time += 20_ms;
  auto setpt = m_profile.Calculate( m_elapsed_time );

  if( m_direction == FORWARD ) {
    m_drive->Drive( frc::ChassisSpeeds{ setpt.velocity, 0_mps, 0_rpm }, false );
  } else {
    m_drive->Drive( frc::ChassisSpeeds{ 0_mps, setpt.velocity, 0_rpm }, false );
  }
}

bool TestProfileMove::IsFinished( void ) {
  return m_profile.IsFinished( m_elapsed_time + 0.25_s );
}