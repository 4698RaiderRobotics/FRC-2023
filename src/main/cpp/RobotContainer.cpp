
#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/StartEndCommand.h>

#include "commands/TestProfileMove.h"
#include "commands/Intake.h"
#include "commands/TargetLimelight.h"
#include "commands/GyroBalance.h"
#include "commands/UpdateOdom.h"
#include "commands/ArmSet.h"
#include "commands/PlaceGamePiece.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        m_drive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
        m_arm.AdjustAngle(arm_angle_axis.GetAxis() * 0.5_deg);
      },
      {&m_drive, &m_arm}));

  // Configure the button bindings
  ConfigureButtonBindings();

  m_chooser.SetDefaultOption(kBalance, kBalance);
  m_chooser.AddOption(kLeave, kLeave);
  m_chooser.AddOption(kPlaceOnly, kPlaceOnly);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here

  // Press the B button to balance the robot on the Charge Station
  frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kCircle)
      .WhileTrue(GyroBalance(&m_drive).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kTriangle)
      .WhileTrue(PlaceGamePiece(&m_drive, &m_arm, &m_grabber, 30_deg).ToPtr());

  (frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kL1) && frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kR1))
      .OnTrue(frc2::InstantCommand([this]
                                   { m_drive.ResetGyro(180_deg); },
                                   {&m_drive})
                  .ToPtr());
  // m_operatorController.RightTrigger().ToggleOnTrue(frc2::StartEndCommand( [this] { m_grabber.Cone( true ); }, [this] { m_grabber.Cone( false ); } ).ToPtr() );
  frc2::JoystickButton(&m_driverController, frc::PS4Controller::Button::kCross)
      .OnTrue(frc2::StartEndCommand([this]
                                    { m_leds.Rainbow(); },
                                    [this]
                                    { m_leds.SetColor(0, 0, 0); })
                  .ToPtr());
  // m_operatorController.LeftTrigger().ToggleOnTrue( frc2::StartEndCommand( [this] { m_grabber.Cube( true ); }, [this] { m_grabber.Cube( false ); } ).ToPtr() );
  m_operatorController.RightTrigger().OnTrue(frc2::InstantCommand([this]
                                                                  { m_grabber.HandleCone(); })
                                                 .ToPtr());

  m_operatorController.LeftTrigger().OnTrue(frc2::InstantCommand([this]
                                                                 { m_grabber.HandleCube(); })
                                                .ToPtr());

  // m_operatorController.RightTrigger().ToggleOnTrue( Intake( &m_grabber, true ).ToPtr() );

  // m_operatorController.LeftTrigger().OnTrue(Intake( &m_grabber, true ).ToPtr() );
  m_operatorController.Y().OnTrue(ArmSet(&m_arm, 12_deg).ToPtr());
  // Hamburger 🍔 Button.
  m_operatorController.Button(8).OnTrue(ArmSet(&m_arm, 25_deg).ToPtr());

  m_operatorController.B().OnTrue(ArmSet(&m_arm, -4_deg).ToPtr());

  m_operatorController.A().OnTrue(ArmSet(&m_arm, -90_deg).ToPtr());

  m_operatorController.X().OnTrue(ArmSet(&m_arm, -35_deg).ToPtr());

  m_operatorController.RightStick().OnTrue(ArmSet(&m_arm, -118_deg).ToPtr());
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  delete m_autoCommand;
  m_autoCommand = nullptr;

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == kBalance)
  {
    m_autoCommand = new BalanceAuto(&m_drive, &m_arm, &m_grabber);
  }
  else if (m_autoSelected == kLeave)
  {
    m_autoCommand = new LeaveAuto(&m_drive, &m_arm, &m_grabber);
  }
  else if (m_autoSelected == kPlaceOnly)
  {
    m_autoCommand = new PlaceOnlyAuto(&m_drive, &m_arm, &m_grabber);
  }

  return m_autoCommand;
}

void RobotContainer::TeleopDataSetup()
{
  //  m_arm.ArmTestSetup();
  //  m_drive.DrivetrainSetup();
  frc::SmartDashboard::PutData(&PDP);
  //  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TeleopDataUpdate()
{
  //  m_arm.ArmTestSetup();
  //  m_drive.DrivetrainSetup();
  //  frc::SmartDashboard::PutData(&PDP);
  // frc::SmartDashboard::PutData(&Compressor);
  //  m_grabber.GrabberTest();
}

void RobotContainer::TestDataSetup()
{
  // Arm Commands and Setup

  frc::SmartDashboard::PutData("Goto -90", new ArmSet(&m_arm, -90_deg));
  frc::SmartDashboard::PutData("Goto 30", new ArmSet(&m_arm, 30_deg));
  frc::SmartDashboard::PutData("Goto 45", new ArmSet(&m_arm, 45_deg));
  frc::SmartDashboard::PutData("Goto -35", new ArmSet(&m_arm, -35_deg));
  // frc::SmartDashboard::PutData( "Run Grabber Motor.", new Intake(&m_grabber, true));
  m_arm.ArmDataSetup();

  m_drive.DrivetrainSetup();
  frc::SmartDashboard::PutData(&PDP);
  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TestDataUpdate()
{
  m_arm.ArmDataUpdate();
  // fmt::print( "TestUpdate" );
  // m_grabber.GrabberTest();
  m_drive.DrivetrainTest();
  m_limelight.LimelightTest();
}