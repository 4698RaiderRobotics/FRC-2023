#include "SwerveModule.h"

#include <cmath>
#include "units/angle.h"
#include "units/base.h"
#include "units/dimensionless.h"
#include <iostream>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>


SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnMotor{ turnMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnEncoder{ absoluteEncoderChannel, absoluteEncoderOffset } {
    m_drivePIDController.SetP(kDriveP);
    m_drivePIDController.SetI(kDriveI);
    m_drivePIDController.SetD(kDriveD);
    m_drivePIDController.SetFF(kDriveFF);
    m_turnPIDController.EnableContinuousInput(-180, 180);
    m_turnPIDController.SetP( kTurnP );   
    m_turnPIDController.SetD( kTurnD );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    const auto state = frc::SwerveModuleState::Optimize( referenceState, m_turnEncoder.GetPosition() );

    // The setpoint rpm for the motor
    units::revolutions_per_minute_t rpm = state.speed / physical::kDriveMetersPerRotation;

    // Distance between the setpoint angle and the current angle in degrees
    units::degree_t dTheta =  state.angle.Degrees() - m_turnEncoder.GetPosition();

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^2 of (the error angle) * the rpm
    units::revolutions_per_minute_t opRPM = rpm * std::pow( units::math::cos( dTheta ).value(), 2 ); 
    
    // The onboard PID controller has a SetReference() function that automatically sets the motor to the correct speed.
    m_drivePIDController.SetReference( opRPM.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    // The software PID controller outputs a value 0 to 1 which must be set using the Set() function of the motor.
    m_turnMotor.Set( m_turnPIDController.Calculate( m_turnEncoder.GetPosition().value(), state.angle.Degrees().value() ) );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    //undo this for odometry
    return { units::meters_per_second_t{ round(m_driveEncoder.GetVelocity() * physical::kDriveMetersPerRotation.value() / 60.0 )}, units::radian_t{ m_turnEncoder.GetPosition() } };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveEncoder.GetPosition() } * physical::kDriveMetersPerRotation, m_turnEncoder.GetPosition()  };
}

void SwerveModule::ModuleSetup() {
    frc::SmartDashboard::PutNumber( "kTurnP", kTurnP );
    frc::SmartDashboard::PutNumber( "kTurnD", kTurnD );
    frc::SmartDashboard::PutNumber( "kDriveP", kDriveP );
    frc::SmartDashboard::PutNumber( "kDriveD", kDriveD );
    frc::SmartDashboard::PutNumber( "kDriveFF", kDriveFF );
}

void SwerveModule::ModuleTest( std::string name) {
    double tP = frc::SmartDashboard::GetNumber( "kTurnP", 0.0 );
    double tD = frc::SmartDashboard::GetNumber( "kTurnD", 0.0 );
    double dP = frc::SmartDashboard::GetNumber( "kDriveP", 0.0 );
    double dD = frc::SmartDashboard::GetNumber( "kDriveD", 0.0 );
    double dFF = frc::SmartDashboard::GetNumber( "kDriveFF", 0.0 );

    if(tP != kTurnP) { kTurnP = tP; }
    if(tD != kTurnD) { kTurnD = tD; }
    if(dP != kDriveP) { kDriveP = dP; }
    if(dD != kDriveD) { kDriveD = dD; }
    if(dFF != kDriveFF) { kDriveFF = dFF; }

    m_drivePIDController.SetP( kDriveP );
    m_drivePIDController.SetD( kDriveD );
    m_drivePIDController.SetFF( kDriveFF );
    m_turnPIDController.SetP( kTurnP );   
    m_turnPIDController.SetD( kTurnD );

    frc::SmartDashboard::PutNumber( name, m_turnEncoder.GetRawPosition() );
}