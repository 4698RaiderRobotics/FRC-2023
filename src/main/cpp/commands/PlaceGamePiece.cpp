// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceGamePiece.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceGamePiece::PlaceGamePiece( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, Limelight *limelight, 
                                units::meter_t distance, units::degree_t angle ) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    ArmSet( angle, arm ),
    TargetLimelight{ drive, limelight },
    frc2::TrapezoidProfileCommand<units::meters>{
          frc::TrapezoidProfile<units::meters>(
              { 3_mps, 3_mps_sq },
              { distance, 0_mps } ),
          [this]( auto setpointState ) {
            frc::ChassisSpeeds speeds;
            speeds.vy = setpointState.velocity;
            m_drive->Drive( speeds, false );
          },
    
          {drive}},
    OpenGrabber( grabber )
  );
  m_timer.Restart();
}

frc2::Command::InterruptionBehavior PlaceGamePiece::GetInterruptionBehavior() const {
  if ( m_timer.HasElapsed( 0.5_s ) ) {
    return frc2::Command::InterruptionBehavior::kCancelSelf;
  } else {
    return frc2::Command::InterruptionBehavior::kCancelIncoming;
  }
}