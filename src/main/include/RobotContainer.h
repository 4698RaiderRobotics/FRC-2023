#pragma once

#include <frc2/command/Command.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/PowerDistribution.h>
#include <frc/Compressor.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

#include "commands/autonomous/SimpleAuto.h"
#include "commands/autonomous/WizzyWiggAuto.h"
#include "commands/autonomous/NoBalanceAuto.h"

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

  void TeleopDataSetup();
  void TeleopDataUpdate();
  void TestDataSetup();
  void TestDataUpdate();

 private:
  // The robot's subsystems and commands are defined here...
  Limelight m_limelight;
  Drivetrain m_drive{ &m_limelight };
  ArmSubsystem m_arm;
  GrabberSubsystem m_grabber;

//  SimpleAuto m_simpleAuto{ &m_drive, &m_arm, &m_grabber, 30_deg };
// WizzyWiggAuto m_wizzyWiggAuto{ &m_drive, &m_arm, &m_grabber };

  frc2::Command *m_autoCommand = nullptr;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Place Cone and Balance";
  const std::string kAutoNameCustom = "Place Cone and Leave";
  const std::string kAutoNameThird = "Just Place Cone";
  std::string m_autoSelected;

  frc::PowerDistribution PDP{0, frc::PowerDistribution::ModuleType::kCTRE};
  frc::Compressor Compressor{9, frc::PneumaticsModuleType::CTREPCM}; 

  frc::PS4Controller m_driverController{ 0 };
  frc2::CommandXboxController m_operatorController{ 1 };

  ControllerAxis vx_axis{ m_driverController, frc::PS4Controller::Axis::kLeftY, true };
  ControllerAxis vy_axis{ m_driverController, frc::PS4Controller::Axis::kLeftX, true };
  ControllerAxis omega_axis{ m_driverController, frc::PS4Controller::Axis::kRightX, true };
  ControllerAxis arm_angle_axis{ m_operatorController, frc::XboxController::Axis::kLeftY, true };

  void ConfigureButtonBindings();
};
