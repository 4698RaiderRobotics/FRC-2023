
#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/StartEndCommand.h>

#include "commands/TestProfileMove.h"
#include "commands/OpenGrabber.h"
#include "commands/TargetLimelight.h"
#include "commands/GyroBalance.h"
#include "commands/UpdateOdom.h"
#include "commands/ArmSet.h"
#include "commands/CloseGrabber.h"
#include "commands/PlaceGamePiece.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] { m_drive.ArcadeDrive( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis() ); }, { &m_drive } ) );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // Press the B button to balance the robot on the Charge Station
  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCircle )
  .WhileTrue( GyroBalance( &m_drive ).ToPtr() );

  // Press the X button to do Limelight targeting w/ Apriltags
//  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCross )
//    .WhileTrue( TargetLimelight( &m_drive, &m_limelight, frc::Pose2d{ -36_in, 22_in, 0_deg } ).ToPtr());

  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCross )
    .WhileTrue( PlaceGamePiece( &m_drive, &m_arm, &m_grabber, 
                  frc::Pose2d{ -42_in, 22_in, 0_deg }, 30_deg ).ToPtr());

  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kL1 )
    .WhileTrue( PlaceGamePiece( &m_drive, &m_arm, &m_grabber, 
                  frc::Pose2d{ -42_in, 0_in, 0_deg }, 30_deg ).ToPtr());

  // Press the Y button to update the odometry with Apriltags
  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kSquare )
  .WhileTrue( UpdateOdom( &m_drive, &m_limelight).ToPtr());

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kRightBumper )
  .OnTrue( OpenGrabber( &m_grabber, 0.0 ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kLeftBumper )
  .WhileTrue( CloseGrabber( &m_grabber ).ToPtr() );

  m_operatorController.RightTrigger().OnTrue( frc2::InstantCommand( [this] { m_grabber.Spin( -0.25 ); }, { &m_grabber } ).ToPtr() );

  m_operatorController.LeftTrigger().ToggleOnTrue( frc2::StartEndCommand( [this] { m_grabber.Spin( 0 ); },
   [this] { m_grabber.Spin( m_grabber.kRollerGripPercent ); }, { &m_grabber } ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kY )
  .WhileTrue( ArmSet( 30_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kB )
  .WhileTrue( ArmSet( 0_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kA )
  .WhileTrue( ArmSet( -90_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kX )
  .WhileTrue( ArmSet( -35_deg, &m_arm ).ToPtr() );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_wizzyWiggAuto;
}

void RobotContainer::TeleopDataSetup() {
//  m_arm.ArmTestSetup();
//  m_drive.DrivetrainSetup();
//  frc::SmartDashboard::PutData(&PDP);
//  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TeleopDataUpdate() {
//  m_arm.ArmTestSetup();
//  m_drive.DrivetrainSetup();
//  frc::SmartDashboard::PutData(&PDP);
  frc::SmartDashboard::PutData(&Compressor);
}


void RobotContainer::TestDataSetup() {
  // Arm Commands and Setup
  frc::SmartDashboard::PutData( "Goto -90", new ArmSet( -90_deg, &m_arm ) );
  frc::SmartDashboard::PutData( "Goto 30", new ArmSet( 30_deg, &m_arm ) );
  frc::SmartDashboard::PutData( "Goto 45", new ArmSet( 45_deg, &m_arm ) );
  frc::SmartDashboard::PutData( "Goto -35", new ArmSet( -35_deg, &m_arm ) );
  m_arm.ArmDataSetup(  );

  m_drive.DrivetrainSetup();
  frc::SmartDashboard::PutData(&PDP);
  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TestDataUpdate() {
  m_arm.ArmDataUpdate( );
  m_grabber.GrabberTest();
  m_drive.DrivetrainTest();
  m_limelight.LimelightTest();
}