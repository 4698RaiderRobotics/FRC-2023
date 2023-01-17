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

  // Press the A button to follow a limelight target
  frc2::JoystickButton( &m_driverController, frc::XboxController::Button::kA)
  .WhileTrue( TargetLimelight( &m_drive, &m_limelight, false).ToPtr() );

  // Press the B button to balance the robot on the Charge Station
  frc2::JoystickButton( &m_driverController, frc::XboxController::Button::kB )
  .WhileTrue( GyroBalance( &m_drive ).ToPtr() );
  // Bress the X button to do Limelight targeting w/ Apriltags
  frc2::JoystickButton( &m_driverController, frc::XboxController::Button::kX )
  .WhileTrue( TargetLimelight( &m_drive, &m_limelight, true).ToPtr());
  frc2::JoystickButton( &m_driverController, frc::XboxController::Button::kY )
  .WhileTrue( UpdateOdom( &m_drive, &m_limelight).ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
