#include "SwerveModule.h"

#include <cmath>
#include "units/angle.h"
#include "units/base.h"
#include "units/dimensionless.h"
#include <iostream>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"


SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnMotor{ turnMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnEncoder{ absoluteEncoderChannel, absoluteEncoderOffset } {

    m_driveMotor.RestoreFactoryDefaults();

    m_drivePIDController.SetP( pidf::kDriveP );
    m_drivePIDController.SetI( pidf::kDriveI );
    m_drivePIDController.SetD( pidf::kDriveD );
    m_drivePIDController.SetFF( pidf::kDriveFF );

    m_turnMotor.RestoreFactoryDefaults();

    m_turnMotor.SetSmartCurrentLimit( 30 );
    m_driveMotor.SetSmartCurrentLimit( 30 );
   
    m_turnPIDController.EnableContinuousInput(-180, 180);
    m_turnPIDController.SetP( pidf::kTurnP );   
    m_turnPIDController.SetD( pidf::kTurnD );

    m_name = fmt::format( "Swerve Module({}-{})", turnMotorChannel, driveMotorChannel );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    const auto state = frc::SwerveModuleState::Optimize(referenceState, m_turnEncoder.GetRollOverPosition());

    // The setpoint rpm for the motor
    units::revolutions_per_minute_t rpm = state.speed / physical::kDriveMetersPerRotation;

    // Distance between the setpoint angle and the current angle in degrees
    units::degree_t dTheta = state.angle.Degrees() - m_turnEncoder.GetRollOverPosition();

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^2 of (the error angle) * the rpm
    units::revolutions_per_minute_t opRPM = rpm * std::pow( units::math::cos( dTheta ).value(), 6 ); 
    
    // The onboard PID controller has a SetReference() function that automatically sets the motor to the correct speed.
    m_drivePIDController.SetReference( opRPM.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );


    // The software PID controller outputs a value 0 to 1 which must be set using the Set() function of the motor.
    double pidOutput = m_turnPIDController.Calculate(m_turnEncoder.GetRollOverPosition().value(), state.angle.Degrees().value());
    m_turnMotor.Set( pidOutput );

    DataLogger::GetInstance().Send( m_name + "/Turn Setpoint", state.angle.Degrees().value() );
    DataLogger::GetInstance().Send( m_name + "/Turn Position", m_turnEncoder.GetRollOverPosition().value() );
    DataLogger::GetInstance().Send( m_name + "/Turn pidoutput", pidOutput );
    // DataLogger::GetInstance().Send( m_name + "/Turn feedforward", feedforwardOutput / 12 );


    DataLogger::GetInstance().Send( m_name + "/Delta Theta", dTheta.value() );
    DataLogger::GetInstance().Send( m_name + "/Desired RPM", rpm.value() );
    DataLogger::GetInstance().Send( m_name + "/Optimized RPM", opRPM.value() );
    DataLogger::GetInstance().Send( m_name + "/Drive Current", m_driveMotor.GetOutputCurrent() );
    DataLogger::GetInstance().Send( m_name + "/Turn Current", m_turnMotor.GetOutputCurrent() );
    DataLogger::GetInstance().Send( m_name + "/Drive Motor Speed", m_driveEncoder.GetVelocity() );
    DataLogger::GetInstance().Send( m_name + "/Turn Motor Speed", m_turnMotorEncoder.GetVelocity() );
}

void SwerveModule::StopMotors( ) {
    m_driveMotor.Set( 0.0 );
    m_turnMotor.Set( 0.0 );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    //undo this for odometry
    return { units::meters_per_second_t{ round(m_driveEncoder.GetVelocity() * physical::kDriveMetersPerRotation.value() / 60.0)}, 
            units::radian_t{ m_turnEncoder.GetRollOverPosition() } };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveEncoder.GetPosition() } *physical::kDriveMetersPerRotation, m_turnEncoder.GetRollOverPosition() };
}

void SwerveModule::ModuleSetup() {
    
    // frc::SmartDashboard::PutNumber( "kDriveP", kDriveP );
    // frc::SmartDashboard::PutNumber( "kDriveD", kDriveD );
    // frc::SmartDashboard::PutNumber( "kDriveFF", kDriveFF );
    // frc::SmartDashboard::PutNumber( "kTurnP", kTurnP );
    // frc::SmartDashboard::PutNumber( "kTurnD", kTurnD );

}

void SwerveModule::ModuleTest( std::string name) {
    // double tP = frc::SmartDashboard::GetNumber( "kTurnP", 0.0 );
    // double tD = frc::SmartDashboard::GetNumber( "kTurnD", 0.0 );
    // double dP = frc::SmartDashboard::GetNumber( "kDriveP", 0.0 );
    // double dD = frc::SmartDashboard::GetNumber( "kDriveD", 0.0 );
    // double dFF = frc::SmartDashboard::GetNumber( "kDriveFF", 0.0 );

    // if(tP != kTurnP) { kTurnP = tP; m_turnPIDController.SetP( kTurnP ); }
    // if(tD != kTurnD) { kTurnD = tD; m_turnPIDController.SetD( kTurnD ); }
    // if(dP != kDriveP) { kDriveP = dP; m_drivePIDController.SetP( kDriveP ); }
    // if(dD != kDriveD) { kDriveD = dD; m_drivePIDController.SetD( kDriveD ); }
    // if(dFF != kDriveFF) { kDriveFF = dFF; m_drivePIDController.SetFF( kDriveFF ); }
    
    frc::SmartDashboard::PutNumber(name, m_turnEncoder.GetRawPosition());
    frc::SmartDashboard::PutNumber( name + " Velocity", m_driveEncoder.GetVelocity() * physical::kDriveMetersPerRotation.value() / 60.0 );
}