// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceGamePiece.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceGamePiece::PlaceGamePiece( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, Limelight *limelight, 
                                units::meter_t distance, units::degree_t angle ) 
    : m_drive{ drive }, m_arm{ arm }, m_grabber{ grabber }, m_limelight{ limelight } {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    TargetLimelight{ m_drive, m_limelight },
    frc2::TrapezoidProfileCommand<units::meters>{
          frc::TrapezoidProfile<units::meters>(
              { 3_mps, 3_mps_sq },
              { distance, 0_mps } ),
          [this]( auto setpointState ) {
            frc::ChassisSpeeds speeds;
            speeds.vy = setpointState.velocity;
            m_drive->Drive( speeds );
          },
     
          {m_drive}},
    ArmSet( angle, m_arm ),
    OpenGrabber( m_grabber ),
    ArmSet( 0_deg, m_arm )
  );
}
