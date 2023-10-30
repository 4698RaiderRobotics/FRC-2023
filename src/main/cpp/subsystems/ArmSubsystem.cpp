// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>

ArmSubsystem::ArmSubsystem() {
    m_rightArm.SetInverted(true);
    m_rightArm.Follow(m_leftArm);
    m_leftArm.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_rightArm.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_leftArm.ConfigNeutralDeadband(0.001);
    m_leftArm.ConfigVoltageCompSaturation(12);
    m_leftArm.EnableVoltageCompensation(true);

    m_wrist.EnableVoltageCompensation(12);

    armAngle = GetArmAngle();
    wristAngle = GetWristAngle();

    m_wristSetpoint.position = wristAngle;
    m_wristGoal = { wristAngle, 0_deg_per_s };
    m_armSetpoint.position = armAngle;
    m_armGoal = { armAngle, 0_deg_per_s };
};

void ArmSubsystem::Periodic() {

    if (!m_armEncoder.IsConnected() ) {
        FRC_ReportError(-111 /*generic error*/, "Arm Absolute Encoder is disconnected.");
        return;
    }

    if ( !m_wristEncoder.IsConnected()) {
        FRC_ReportError(-111 /*generic error*/, "Wrist Absolute Encoder is disconnected.");
        return;
    }

    armAngle = GetArmAngle();
    wristAngle = GetWristAngle();
    if (armAngle > 180_deg) {
        m_armEncoder.SetPositionNegative();
    }
    if (wristAngle > 180_deg) {
        m_wristEncoder.SetPositionNegative();
    }


    armAngle = GetArmAngle();
    wristAngle = GetWristAngle();

    // Holds arm in position after enabled
    if ( disabled && frc::DriverStation::IsEnabled() ) {
        m_wristSetpoint.position = wristAngle;
        m_armSetpoint.position = armAngle;
        m_armAngleGoal = armAngle;
        m_wristAngleGoal = wristAngle;
        disabled = false;
    } else if ( !disabled && frc::DriverStation::IsDisabled() ) {
        disabled = true;
    }

    // Update Shuffleboard with values
    // ArmData();

    // Checks for unplugged absolute encoder on the arm
    if (armAngle > 120_deg || armAngle < -140_deg ) {
        FRC_ReportError	( -111 /*generic error*/, "Arm Absolute Encoder out of bounds." );
        return;
    }

    m_armGoal = { m_armAngleGoal, 0_deg_per_s };
    m_wristGoal = { m_wristAngleGoal, 0_deg_per_s };

    frc::TrapezoidProfile<units::degrees> m_wristProfile{ m_wristConstraints, m_wristGoal, m_wristSetpoint };
    m_wristSetpoint = m_wristProfile.Calculate(dt);


    units::degree_t alpha = 90_deg + wristAngle + armAngle;

    double wristOutput = wristPID.Calculate(wristAngle.value(), m_wristSetpoint.position.value());
    double wristFeedforwardOut = wristFeedforward.Calculate(alpha, m_wristSetpoint.velocity).value();

    // frc::SmartDashboard::PutNumber("Wrist PID Output", wristOutput);
    // frc::SmartDashboard::PutNumber("Wrist Feedforward Output", wristFeedforwardOut);

    m_wrist.Set(wristOutput + wristFeedforwardOut / 12);


    // Arm Control
    frc::TrapezoidProfile<units::degrees> m_armProfile{ m_armConstraints, m_armGoal, m_armSetpoint };
    m_armSetpoint = m_armProfile.Calculate(dt);

    double armOutput = armPID.Calculate(armAngle.value(), m_armSetpoint.position.value());
    double armFeedforwardOut = armFeedforward.Calculate(m_armSetpoint.position, m_armSetpoint.velocity).value() + pidf::kArmGWrist * units::math::cos(alpha).value();

    // frc::SmartDashboard::PutNumber("Arm PID Output", armOutput);
    // frc::SmartDashboard::PutNumber("Arm Feedforward Output", armFeedforwardOut);

    m_leftArm.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, armOutput + armFeedforwardOut / 12.0);
}

// Changed the setpoint of the arm
void ArmSubsystem::GotoAngle(units::degree_t armAngle, units::degree_t wristAngle) {
    // fmt::print( "GoToArmAngle = {}, GoToWristAngle = {} \n", armAngle, wristAngle );
    m_armAngleGoal = armAngle;

    m_wristAngleGoal = wristAngle;

        // Range check angle
    if (m_armAngleGoal > max_angle) m_armAngleGoal = max_angle;
    if (m_armAngleGoal < min_angle) m_armAngleGoal = min_angle;

    if (m_wristAngleGoal > maxWristAngle) m_wristAngleGoal = maxWristAngle;
    if (m_wristAngleGoal < minWristAngle) m_wristAngleGoal = minWristAngle;
    // fmt::print( "GoalArm = {}, GoalWrist = {}\n", m_armAngleGoal, m_wristAngleGoal );
}

// Allows setpoint of arm to be nudged
void ArmSubsystem::AdjustArmAngle(units::degree_t delta_angle) {
    m_armAngleGoal += delta_angle;

    //m_wristAngle += delta_angle;
    // fmt::print( "AdjustAngle = {}\n", delta_angle );

        // Range check angle
    if (m_armAngleGoal > max_angle) m_armAngleGoal = max_angle;
    if (m_armAngleGoal < min_angle) m_armAngleGoal = min_angle;
}

void ArmSubsystem::AdjustWristAngle(units::degree_t delta_angle) {
    m_wristAngleGoal += delta_angle;

    if (m_wristAngleGoal > maxWristAngle) m_wristAngleGoal = maxWristAngle;
    if (m_wristAngleGoal < minWristAngle) m_wristAngleGoal = minWristAngle;
}

void ArmSubsystem::ArmOn( double armSpeed, double wristSpeed ) {
    m_leftArm.Set( ctre::phoenix::motorcontrol::ControlMode::PercentOutput, armSpeed );
    m_wrist.Set( wristSpeed );
}

bool ArmSubsystem::Finished( ) {
    return (units::math::abs(m_armEncoder.GetCountingPosition() - m_armAngleGoal) < 
            physical::kArmAngleError && units::math::abs(wristAngle - m_wristAngleGoal) < physical::kArmAngleError);
}

void ArmSubsystem::ArmDataSetup( ) {
    /*
        frc::SmartDashboard::PutNumber("kG", armFeedforward.kG.value());
    frc::SmartDashboard::PutNumber("kP", kP);
    frc::SmartDashboard::PutNumber("kD", kD);
    frc::SmartDashboard::PutNumber("kV", kV);
    frc::SmartDashboard::PutNumber("kS", kS);

    frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                                                    * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );
*/
}

void ArmSubsystem::ArmDataUpdate( ) {
    /*
    double g = frc::SmartDashboard::GetNumber( "kG", 0.0 );
    double p = frc::SmartDashboard::GetNumber( "kP", 0.0 );
    double d = frc::SmartDashboard::GetNumber( "kD", 0.0 );
    double v = frc::SmartDashboard::GetNumber( "kV", 0.0 );
    double s = frc::SmartDashboard::GetNumber( "kS", 0.0 );

    if (g != armFeedforward.kG.value()) { armFeedforward.kG = units::volt_t{ g }; }
    if( p != kP ) { kP = p; pid.SetP( kP ); }
    if( d != kD ) { kD = d; pid.SetD( kD ); }
    if (v != kV) { kV = v; armFeedforward.kV = units::unit_t<frc::ArmFeedforward::kv_unit>{ kV }; }
    if (s != kS) { kS = s; armFeedforward.kS = units::volt_t{ kS }; }

    frc::SmartDashboard::PutNumber( "Arm Position", m_enc.GetPosition().value() );
    frc::SmartDashboard::PutNumber( "Arm Velocity", m_left.GetSensorCollection().GetIntegratedSensorVelocity() 
                                                    * physical::tics_per_100ms_to_deg_per_s * physical::kArmGearRatio );
    

    frc::SmartDashboard::PutNumber( "Arm Raw Pos", m_enc.GetRawPosition() );
    */
}

void ArmSubsystem::ArmData() {

    frc::SmartDashboard::PutNumber("Goal Arm Angle", m_armSetpoint.position.value());
    frc::SmartDashboard::PutNumber("Goal Arm Velocity", m_armSetpoint.velocity.value());
    frc::SmartDashboard::PutNumber("Current Arm Angle", armAngle.value());
    frc::SmartDashboard::PutNumber("Current Arm Velocity", m_leftArm.GetSensorCollection().GetIntegratedSensorVelocity()
        * 3600.0 / 2048.0 * 12.0 / 58.0 * 18.0 / 58.0 * 15.0 / 26.0);
    frc::SmartDashboard::PutNumber("ArmPosition", GetArmAngle().value());
    frc::SmartDashboard::PutNumber("ArmRawPosition", m_armEncoder.GetRawPosition());

    frc::SmartDashboard::PutNumber("Goal Wrist Angle", m_wristSetpoint.position.value());
    frc::SmartDashboard::PutNumber("Goal Wrist Velocity", m_wristSetpoint.velocity.value());
    frc::SmartDashboard::PutNumber("Current Wrist Angle", wristAngle.value());
    frc::SmartDashboard::PutNumber("Current Wrist Velocity", m_enc.GetVelocity() * 0.0129 * 6.0);
    frc::SmartDashboard::PutNumber("WristPosition", GetWristAngle().value());
    frc::SmartDashboard::PutNumber("WristRawPosition", m_wristEncoder.GetRawPosition());
}

units::degree_t ArmSubsystem::GetArmAngle() {
    return m_armEncoder.GetCountingPosition();
}

units::degree_t ArmSubsystem::GetWristAngle() {
    return m_wristEncoder.GetCountingPosition() * physical::kWristEncoderGearRatio;
}