#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>

#include <frc/PowerDistribution.h>
#include <frc/Compressor.h>

#include "commands/ExampleCommand.h"
#include "commands/TargetLimelight.h"
#include "commands/GyroBalance.h"
#include "commands/UpdateOdom.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"
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

  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;

  frc::PowerDistribution PDP{0, frc::PowerDistribution::ModuleType::kCTRE};
  frc::Compressor Compressor{9, frc::PneumaticsModuleType::REVPH}; 

  frc::XboxController m_driverController{ 0 };
  ControllerAxis vx_axis{ m_driverController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis vy_axis{ m_driverController, frc::XboxController::Axis::kLeftX, true };
  ControllerAxis omega_axis{ m_driverController, frc::XboxController::Axis::kRightX, true };

  void ConfigureButtonBindings();
};
