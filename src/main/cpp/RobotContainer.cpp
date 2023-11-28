
#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc/Timer.h>

#include <cmath>
#include <iostream>

#include "commands/TestProfileMove.h"
#include "commands/Intake.h"
#include "commands/TargetLimelight.h"
#include "commands/GyroBalance.h"
#include "commands/UpdateOdom.h"
#include "commands/ArmSet.h"
#include "commands/LedCommands/Idle.h"
#include "commands/PlaceGamePiece.h"
#include "commands/LedCommands/Rainbow.h"
#include "commands/PlaceMid.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
   },
    { &m_drive }
    ));

  m_arm.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_arm.AdjustArmAngle(arm_angle_axis.GetAxis() * 0.5_deg);
      m_arm.AdjustWristAngle(wrist_angle_axis.GetAxis() * 0.5_deg);
    },
    { &m_arm }
    ));

  //m_leds.SetDefaultCommand(std::move(m_ledCommand));
  /*m_leds.SetDefaultCommand(Idle(&m_leds).IgnoringDisable(true).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf).HandleInterrupt([this] {
    std::cout << "Interrupted \n";
    }));*/
    //m_leds.SetDefaultCommand();
  // Configure the button bindings
  ConfigureButtonBindings();

  m_chooser.SetDefaultOption( kBalance, kBalance );
  m_chooser.AddOption( kLeave, kLeave );
  m_chooser.AddOption( kPlaceOnly, kPlaceOnly );
  m_chooser.AddOption( kDoNothing, kDoNothing );
  m_chooser.AddOption( kTrajPractice, kTrajPractice );
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // Press the B button to balance the robot on the Charge Station
  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kCircle)
  //   .OnTrue(GyroBalance(&m_drive).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kTriangle)
  //   .OnTrue(frc2::InstantCommand([this] {m_arm.ArmOn(-0.75, 0.5); }, { &m_arm }).ToPtr().AndThen(frc2::WaitCommand(0.4_s).ToPtr())
  //   .AndThen(frc2::InstantCommand([this] {m_arm.ArmOn(0.0, 0.0); }, { &m_arm }).ToPtr()));

  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kTriangle)
  //   .WhileTrue(PlaceGamePiece(&m_drive, &m_arm, &m_grabber, 30_deg).ToPtr());

  (frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kL1) && frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kR1))
    .OnTrue(frc2::InstantCommand([this] { m_drive.ResetGyro(180_deg); },
      { &m_drive })
      .ToPtr());
  // m_operatorController.RightTrigger().ToggleOnTrue(frc2::StartEndCommand( [this] { m_grabber.Cone( true ); }, [this] { m_grabber.Cone( false ); } ).ToPtr() );
  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kCross)
  //   .OnTrue(frc2::RunCommand([this] { m_leds.Sinusoidal_Pulse(frc::Color::kFirstBlue, 5_s); std::cout << "run\n";}, { &m_leds }).ToPtr());
  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kCross);
  // frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kSquare)
  //  .OnTrue(frc2::InstantCommand([this] { m_leds.SetAll(0, 0, 0); }, { &m_leds }).ToPtr());
  m_operatorController.RightTrigger().OnTrue(frc2::InstantCommand([this] { m_grabber.HandleCube(); }, { &m_grabber, &m_arm, &m_leds }).ToPtr());

  m_operatorController.LeftTrigger().OnTrue(frc2::InstantCommand([this] { m_grabber.HandleCone(); }, { &m_grabber, &m_arm, &m_leds })
    .ToPtr());

  //m_operatorController.Y().OnTrue(ArmSet(&m_arm, physical::kArmLowerPlaceHeight, physical::kWristLowerPlaceHeight, 0.3).ToPtr());

  // Hamburger 🍔 Button.
  // m_operatorController.Button(8).OnTrue(ArmSet(&m_arm, 25_deg, -100_deg, 0.3).ToPtr());

  m_operatorController.Y().OnTrue(ArmSet(&m_arm, physical::kArmUpperPlaceHeight, physical::kWristUpperPlaceHeight, 0.3, true).ToPtr());

  m_operatorController.A().OnTrue(ArmSet(&m_arm, -90_deg, 12_deg, 0.3, true).ToPtr());

  m_operatorController.B().OnTrue(PlaceMid( &m_arm, &m_grabber ).ToPtr() );

  m_operatorController.RightStick().OnTrue(ArmSet(&m_arm, -118_deg, 61_deg, 0.3, true).ToPtr());

}
void RobotContainer::RunLedCommand() {

}
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  delete m_autoCommand;
  m_autoCommand = nullptr;

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == kBalance) {
    m_autoCommand = new BalanceAuto(&m_drive, &m_arm, &m_grabber, &m_leds);
  }
  else if (m_autoSelected == kLeave) {
    m_autoCommand = new LeaveAuto(&m_drive, &m_arm, &m_grabber, &m_leds);
  }
  else if (m_autoSelected == kPlaceOnly) {
    m_autoCommand = new PlaceOnlyAuto(&m_drive, &m_arm, &m_grabber, &m_leds);
  }
  else if ( m_autoSelected == kDoNothing ) {
    m_autoCommand = new DoNothingAuto( &m_drive, &m_arm, &m_grabber, &m_leds );
  } else if (m_autoSelected == kTrajPractice) {
    m_autoCommand = new TrajPracticeAuto(&m_drive, &m_arm, &m_grabber, &m_leds);
  }

  return m_autoCommand;
}

void RobotContainer::TeleopDataSetup() {
  //  m_arm.ArmTestSetup();
  //  m_drive.DrivetrainSetup();
  // frc::SmartDashboard::PutData(&PDP);
  //  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TeleopDataUpdate() {
  //  m_arm.ArmTestSetup();
  //  m_drive.DrivetrainSetup();
  //  frc::SmartDashboard::PutData(&PDP);
  // frc::SmartDashboard::PutData(&Compressor);
  //  m_grabber.GrabberTest();
}

void RobotContainer::TestDataSetup() {
  // Arm Commands and Setup

  //frc::SmartDashboard::PutData("Goto -90", new ArmSet(&m_arm, -90_deg));
  //frc::SmartDashboard::PutData("Goto 30", new ArmSet(&m_arm, 30_deg));
  //frc::SmartDashboard::PutData("Goto 45", new ArmSet(&m_arm, 45_deg));
  //frc::SmartDashboard::PutData("Goto -35", new ArmSet(&m_arm, -35_deg));
  // frc::SmartDashboard::PutData( "Run Grabber Motor.", new Intake(&m_grabber, true));
  m_arm.ArmDataSetup();

  m_drive.DrivetrainSetup();
  // frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TestDataUpdate() {
  // m_arm.ArmDataUpdate();
  // fmt::print( "TestUpdate" );
  // m_grabber.GrabberTest();
  // m_drive.DrivetrainTest();
  // m_limelight.LimelightTest();
}