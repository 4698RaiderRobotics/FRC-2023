#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>

#include "commands/ExampleCommand.h"
#include "commands/TargetLimelight.h"
#include "commands/GyroBalance.h"
#include "commands/UpdateOdom.h"
#include "commands/ArmSet.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"
#include "ControllerAxis.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void TestSetup();

  void TestMode();

 private:
  // The robot's subsystems and commands are defined here...
  Drivetrain m_drive;
  Limelight m_limelight;
  ArmSubsystem m_arm;
  GrabberSubsystem m_grabber;

  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;

  frc::XboxController m_driverController{ 0 };
  frc::XboxController m_operatorController{ 1 };

  ControllerAxis vx_axis{ m_driverController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis vy_axis{ m_driverController, frc::XboxController::Axis::kLeftX, true };
  ControllerAxis omega_axis{ m_driverController, frc::XboxController::Axis::kRightX, true };

  void ConfigureButtonBindings();
};
