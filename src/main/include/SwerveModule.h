#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>

#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/controller/PIDController.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>


#include "AbsoluteEncoder.h"
#include "Constants.h"

// Class for each swerve module on the robot
class SwerveModule {
    public:
        SwerveModule( const int turnMotorChannel, const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset );

        void SetDesiredState( const frc::SwerveModuleState& state );

        frc::SwerveModuleState GetState( void );
        frc::SwerveModulePosition GetPosition ( void );

        void ModuleSetup();

        void ModuleTest( std::string name );
    private:
        rev::CANSparkMax m_driveMotor;
        rev::CANSparkMax m_turnMotor;

        rev::SparkMaxRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
        AbsoluteEncoder m_turnEncoder;

        // The drive motor uses an onboard PID controller (rev::SparkMaxPIDController). 
        // The motor is automatically set by the PID controller.
        rev::SparkMaxPIDController m_drivePIDController = m_driveMotor.GetPIDController();

        // The turn motor uses the software PID controller (frc2::PIDController). 
        // The motor needs to be set with the Set() function with the PID controller's output.
        frc2::PIDController m_turnPIDController{ kTurnP, kTurnI, kTurnD };

        double kTurnP = 0.005;
        double kTurnI = 0;
        double kTurnD = 0;

        double kDriveP = 0.0001;
        double kDriveI = 0;
        double kDriveD = 0;
        double kDriveFF = 0.0002;
};