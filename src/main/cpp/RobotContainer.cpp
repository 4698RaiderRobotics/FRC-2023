
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
  // frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCircle )
  // .WhileTrue( GyroBalance( &m_drive ).ToPtr() );

  // Press the X button to do Limelight targeting w/ Apriltags
//  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCross )
//    .WhileTrue( TargetLimelight( &m_drive, &m_limelight, frc::Pose2d{ -36_in, 22_in, 0_deg } ).ToPtr());

  // frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCross )
  //   .WhileTrue( PlaceGamePiece( &m_drive, &m_arm, &m_grabber, 
  //                 frc::Pose2d{ -42_in, 22_in, 0_deg }, 30_deg ).ToPtr());

  // frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kL1 )
  //   .WhileTrue( PlaceGamePiece( &m_drive, &m_arm, &m_grabber, 
  //                 frc::Pose2d{ -42_in, 0_in, 0_deg }, 30_deg ).ToPtr());

  // Press the Y button to update the odometry with Apriltags
  // frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kSquare )
  // .WhileTrue( UpdateOdom( &m_drive, &m_limelight).ToPtr());

  m_operatorController.RightBumper().OnTrue( OpenGrabber( &m_grabber, 0.0 ).ToPtr() );

  m_operatorController.LeftBumper().OnTrue( CloseGrabber( &m_grabber ).ToPtr() );

  m_operatorController.RightTrigger().OnTrue( frc2::InstantCommand( [this] { m_grabber.Spin( -0.25 ); }, { &m_grabber } ).ToPtr() );

  m_operatorController.LeftTrigger().OnTrue( frc2::InstantCommand( [this] { m_grabber.Toggle( ); }, { &m_grabber } ).ToPtr() );

  m_operatorController.Y().OnTrue( ArmSet( &m_arm, 30_deg ).ToPtr() );

  m_operatorController.B().OnTrue( ArmSet( &m_arm, 0_deg ).ToPtr() );

  m_operatorController.A().OnTrue( ArmSet( &m_arm, -90_deg ).ToPtr() );

  m_operatorController.X().OnTrue( ArmSet( &m_arm, -35_deg ).ToPtr() );

  m_operatorController.RightStick().OnTrue( ArmSet( &m_arm, -118_deg ).ToPtr() );

  // frc2::JoystickButton( &m_operatorController, frc::XboxController::Axis::kLeftY )
  // .WhileTrue( frc2::RunCommand( [this] { m_arm.AdjustAngle( m_operatorController.GetLeftY() * 15_deg ); }, { &m_arm } ).ToPtr() );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  delete m_autoCommand;

  m_autoCommand = new WizzyWiggAuto( &m_drive, &m_arm, &m_grabber );

  return m_autoCommand;
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
  frc::SmartDashboard::PutData( "Goto -90", new ArmSet( &m_arm, -90_deg ) );
  frc::SmartDashboard::PutData( "Goto 30", new ArmSet( &m_arm, 30_deg ) );
  frc::SmartDashboard::PutData( "Goto 45", new ArmSet( &m_arm, 45_deg ) );
  frc::SmartDashboard::PutData( "Goto -35", new ArmSet( &m_arm, -35_deg ) );
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