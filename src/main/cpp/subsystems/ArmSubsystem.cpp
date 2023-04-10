// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>

#include "subsystems/ArmSubsystem.h"
#include "commands/ArmSet.h"

ArmSubsystem::ArmSubsystem() {
    SetName( "ArmSubsystem" );

    m_right.SetInverted( true );
    m_right.Follow( m_left );
    m_left.SetNeutralMode( ctre::phoenix::motorcontrol::Brake );
    m_right.SetNeutralMode( ctre::phoenix::motorcontrol::Brake );

    m_setpoint.position = m_enc.GetPosition();
};

void ArmSubsystem::Periodic() {
    if ( disabled && frc::DriverStation::IsEnabled() ) {
        m_setpoint.position = m_enc.GetPosition();
        m_angle = m_enc.GetPosition();
        disabled = false;
    } else if ( !disabled && frc::DriverStation::IsDisabled() ) {
        disabled = true;
    }
    
    units::degree_t measurement = m_enc.GetPosition();
    if ( measurement > 120_deg || measurement < -140_deg ) {
        FRC_ReportError	( -111 /*generic error*/, "Arm Absolute Encoder out of bounds." );
        return;
    }

    m_goal = { m_angle, 0_deg_per_s };

    frc::TrapezoidProfile<units::degrees> m_profile{ m_constraints, m_goal, m_setpoint };
    m_setpoint = m_profile.Calculate( dt );

    frc::SmartDashboard::PutNumber( "Arm Setpoint Position", m_setpoint.position.value() );
    frc::SmartDashboard::PutNumber( "Arm Setpoint Velocity", m_setpoint.velocity.value() );

    double output = pid.Calculate( measurement.value(), m_setpoint.position.value());
    double feedforwardOut = feedforward.Calculate( m_setpoint.position, m_setpoint.velocity ).value();
    frc::SmartDashboard::PutNumber( "Arm Voltage", output * 12 + feedforwardOut );

    m_left.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output + feedforwardOut / 12 );
}

void ArmSubsystem::GotoAngle( units::degree_t angle ) {
    m_angle = angle;

        // Range check angle
    if( m_angle > max_angle ) m_angle = max_angle;
    if( m_angle < min_angle ) m_angle = min_angle;
}

void ArmSubsystem::AdjustAngle( units::degree_t delta_angle ) {
    
    m_angle += delta_angle;

        // Range check angle
    if( m_angle > max_angle ) m_angle = max_angle;
    if( m_angle < min_angle ) m_angle = min_angle;
}

bool ArmSubsystem::Finished( ) {
    return units::math::abs( m_enc.GetPosition() - m_angle ) < physical::kArmAngleError;
}

void ArmSubsystem::ArmDataSetup( std::string_view tab ) {
    auto &arm_tab = frc::Shuffleboard::GetTab( tab );

    auto &pid_layout = arm_tab.GetLayout( "Feed Forward", frc::BuiltInLayouts::kList ).WithSize( 2, 5 );

    sb_map["kG"] = pid_layout.Add( "kG", feedforward.kG.value() ).GetEntry();
    sb_map["kV"] = pid_layout.Add( "kV", feedforward.kV.value() ).GetEntry();
    sb_map["kS"] = pid_layout.Add( "kS", feedforward.kS.value() ).GetEntry();
    sb_map["kP"] = pid_layout.Add( "kP", pid.GetP() ).GetEntry();
    sb_map["kD"] = pid_layout.Add( "kD", pid.GetD() ).GetEntry();

    sb_map["Arm Position"] = arm_tab.Add( "Arm Position", m_enc.GetPosition().value() ).WithSize(2,2).GetEntry();
    sb_map["Arm Velocity"] = arm_tab.Add( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                         * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio ).WithSize(2,2).GetEntry();
    sb_map["Arm Raw Pos"] = arm_tab.Add( "Arm Raw Pos", m_enc.GetRawPosition() ).WithSize(2,2).GetEntry();

    static ArmSet goto90( this, -90_deg );
    arm_tab.Add( "Goto -90", goto90 ).WithSize(2,2);
    // arm_tab.Add( "Goto 30", ArmSet( this, 30_deg ) );
    // arm_tab.Add( "Goto 45", new ArmSet( this, 45_deg ) );
    // arm_tab.Add( "Goto -35", new ArmSet( this, -35_deg ) );

//    frc::SmartDashboard::PutNumber("kG", feedforward.kG.value() );
    // frc::SmartDashboard::PutNumber("kP", kP);
    // frc::SmartDashboard::PutNumber("kD", kD);
    // frc::SmartDashboard::PutNumber("kV", kV);
    // frc::SmartDashboard::PutNumber("kS", kS);

    // frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    // frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
    //                                                 * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );

}

void ArmSubsystem::ArmDataUpdate( ) {

    double g = sb_map["kG"]->GetDouble( 0.0 );
    double p = sb_map["kP"]->GetDouble( 0.0 );
    double d = sb_map["kD"]->GetDouble( 0.0 );
    double v = sb_map["kV"]->GetDouble( 0.0 );
    double s = sb_map["kS"]->GetDouble( 0.0 );

    // double g = frc::SmartDashboard::GetNumber( "kG", 0.0 );
    // double p = frc::SmartDashboard::GetNumber( "kP", 0.0 );
    // double d = frc::SmartDashboard::GetNumber( "kD", 0.0 );
    // double v = frc::SmartDashboard::GetNumber( "kV", 0.0 );
    // double s = frc::SmartDashboard::GetNumber( "kS", 0.0 );

    if( g != feedforward.kG.value() ) { feedforward.kG = units::volt_t{ g }; }
    if( p != kP ) { kP = p; pid.SetP( kP ); }
    if( d != kD ) { kD = d; pid.SetD( kD ); }
    if( v != kV ) { kV = v; feedforward.kV = units::unit_t<frc::ArmFeedforward::kv_unit>{ kV }; }
    if( s != kS ) { kS = s; feedforward.kS = units::volt_t{ kS }; }

    sb_map[ "Arm Position" ]->SetDouble( m_enc.GetPosition().value() );
    sb_map[ "Arm Velocity" ]->SetDouble( m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                                                    * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );
    sb_map[ "Arm Raw Pos" ]->SetDouble( m_enc.GetRawPosition() );
    
}