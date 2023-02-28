#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  

  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] { m_drive.Drive( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis() ); }, { &m_drive } ) );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // Press the B button to balance the robot on the Charge Station
  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCircle )
  .WhileTrue( GyroBalance( &m_drive ).ToPtr() );

  // Press the X button to do Limelight targeting w/ Apriltags
  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kCross )
  .WhileTrue( TargetLimelight( &m_drive, &m_limelight ).ToPtr());

  // Press the Y button to update the odometry with Apriltags
  frc2::JoystickButton( &m_driverController, frc::PS4Controller::Button::kSquare )
  .WhileTrue( UpdateOdom( &m_drive, &m_limelight).ToPtr());

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kRightBumper )
  .WhileTrue( frc2::InstantCommand( [this] { m_grabber.Open(); }, { &m_grabber } ).ToPtr() );

  m_operatorController.RightTrigger().WhileTrue( frc2::RunCommand( [this] { m_grabber.Spin( -0.25 ); }, { &m_grabber } ).ToPtr() );

  m_operatorController.LeftTrigger().OnTrue( frc2::InstantCommand( [this] { m_grabber.Spin( 0.25 ); }, { &m_grabber } ). ToPtr() )
  .OnFalse( frc2::InstantCommand( [this] { m_grabber.Spin( 0 ); }, { &m_grabber } ). ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kLeftBumper )
  .WhileTrue( CloseGrabber( &m_grabber ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kY )
  .WhileTrue( ArmSet( 30_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kB )
  .WhileTrue( ArmSet( 45_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kA )
  .WhileTrue( ArmSet( -90_deg, &m_arm ).ToPtr() );

  frc2::JoystickButton( &m_operatorController, frc::XboxController::Button::kX )
  .WhileTrue( ArmSet( -35_deg, &m_arm ).ToPtr() );

  //(m_operatorController.LeftBumper() && m_operatorController.A()).OnTrue( PlaceGamePiece( &m_drive, &m_arm, &m_grabber, &m_limelight, -0.56_m, -90_deg ).ToPtr() );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_simpleAuto;
}

void RobotContainer::TestSetup() {
  m_arm.ArmTestSetup();
  //m_drive.DrivetrainSetup();
  frc::SmartDashboard::PutData(&PDP);
  frc::SmartDashboard::PutData(&Compressor);
}

void RobotContainer::TestMode() {
  //m_arm.ArmTest();
  //m_grabber.GrabberTest();
  //m_drive.DrivetrainTest();
  m_limelight.LimelightTest();
  frc::SmartDashboard::PutNumber( "Joystick", m_driverController.GetLeftX() );
}