// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] { m_drive.Drive( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis() ); }, { &m_drive } ) );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton( &m_driverController, frc::XboxController::Button::kA)
  .WhileTrue( TargetLimelight( &m_drive, &m_limelight ).ToPtr() );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
