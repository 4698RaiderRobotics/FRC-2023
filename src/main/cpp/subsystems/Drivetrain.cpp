
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"

Drivetrain::Drivetrain( Limelight *ll ) : m_limelight{ll} {
  ResetGyro( 180_deg );

  for( int n=0; n<9; ++n ) {
    redAllianceGridPoints[n] = { 14.3_m, 20_in + 22_in*n, 0_deg };
    blueAllianceGridPoints[n] = { 2.25_m, 20_in + 22_in*n, 180_deg };
  }

  frc::SmartDashboard::PutData("Field", &m_field);
}

// Drives with joystick inputs
// This takes -1 to 1 inputs
void Drivetrain::ArcadeDrive( double xSpeed, double ySpeed, double omegaSpeed, bool operatorRelative ) {
    auto x = xSpeed * physical::kMaxDriveSpeed;
    auto y = ySpeed * physical::kMaxDriveSpeed;
    auto omega = omegaSpeed * physical::kMaxTurnSpeed;

    frc::ChassisSpeeds speeds{ x, y , omega };

    if( operatorRelative ) {
        // fmt::print( "Gyro Reading = {}\n", units::degree_t{ m_gyro.GetYaw() } );
        // fmt::print( "Field Angle = {}\n", m_odometry.GetEstimatedPosition().Rotation().Degrees() );

        // fmt::print( "  Drivetrain::ArcadeDrive operator angle = {} - {} = {}\n", 
        //                units::degree_t{ m_gyro.GetYaw() }, m_gyro_operator_offset,
        //                units::degree_t{ m_gyro.GetYaw() } - m_gyro_operator_offset );
        speeds = speeds.FromFieldRelativeSpeeds( speeds.vx, speeds.vy, speeds.omega, 
                                                 units::degree_t{ m_gyro.GetYaw() } - m_gyro_operator_offset );
    }

    Drive( speeds, false );
}

void Drivetrain::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    //m_field.SetRobotPose(m_odometry.GetPose());
    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    auto states = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
                    speeds.vx, speeds.vy, speeds.omega, m_odometry.GetEstimatedPosition().Rotation() ) :
                    speeds );
    m_kinematics.DesaturateWheelSpeeds( &states, physical::kMaxDriveSpeed );

    auto [ fl, fr, bl, br ] = states;

    // Sets each SwerveModule to the correct SwerveModuleState
    m_frontLeft.SetDesiredState( fl );
    m_frontRight.SetDesiredState( fr );
    m_backLeft.SetDesiredState( bl );
    m_backRight.SetDesiredState( br );

 //   wpi::array opStates = { m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(), m_backRight.GetState() };
    // Displays the SwerveModules current position
//    swerve_display.SetState( opStates );

    // Updates the odometry of the robot given the SwerveModules' states
/**
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(), m_backRight.GetPosition() 
    });
    frc::Pose2d o_Pose = m_odometry.GetPose();
    double oo_Pose[]  {
        o_Pose.X().value(),
        o_Pose.Y().value(),
        o_Pose.Rotation().Degrees().value()
    };
    frc::SmartDashboard::PutNumberArray("oo_Pose", oo_Pose);
*/
}

// Drives a path given a trajectory state
void Drivetrain::DriveTrajectory( frc::Trajectory::State trajectoryState, const frc::Rotation2d &robotHeading ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetEstimatedPosition(), trajectoryState, robotHeading );

    // fmt::print("trajectory RobotPose ({}, {}, {}), TrajPose ({}, {}, {}), speeds = {}, {}, {}\n", 
    //            m_odometry.GetEstimatedPosition().X(), m_odometry.GetEstimatedPosition().Y(), m_odometry.GetEstimatedPosition().Rotation().Degrees(), 
    //            trajectoryState.pose.X(), trajectoryState.pose.Y(), trajectoryState.pose.Rotation().Degrees(),
    //            adjustedSpeeds.vx, adjustedSpeeds.vy, adjustedSpeeds.omega );

    Drive( adjustedSpeeds, false );
}

void Drivetrain::Periodic( ) {
    // Updates the odometry of the robot given the SwerveModules' states
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(), m_backRight.GetPosition() 
    });

    frc::Pose2d visionPose;
    units::second_t timestamp;
    if( m_limelight->getFieldAprilTagPose( visionPose, timestamp ) ) {

        if( frc::DriverStation::IsEnabled() && m_averagingPose ) {
                // Stop averaging if the DS is enabled.
            m_averagingPose = false;
        }
        if ( frc::DriverStation::IsDisabled() ) {
            if ( visionPose.X() < 7.5_m ) {
                fmt::print( "Blue Alliance\n");
                m_gyro_operator_offset = 360_deg - units::degree_t{ m_gyro.GetYaw() } - visionPose.Rotation().Degrees();
            } else {
                fmt::print( "Red Alliance\n");
                m_gyro_operator_offset = 180_deg - units::degree_t{ m_gyro.GetYaw() } - visionPose.Rotation().Degrees();
            }

            m_odometry.ResetPosition( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
            {
                m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                m_backLeft.GetPosition(), m_backRight.GetPosition() 
            }, visionPose );
        
        //if( m_noValidPose && frc::DriverStation::IsDisabled() ) {
           // AverageVisionPose( visionPose, timestamp );
        } else if ( frc::DriverStation::IsTeleopEnabled() ) {
            if( m_odometry.GetEstimatedPosition().Translation().Distance( visionPose.Translation() ) < 1_m ) {
                // Only update if the vision pose is within 1m of the current pose
                m_odometry.AddVisionMeasurement( visionPose, timestamp );
            }
        }
    }

    frc::Pose2d estm_pose = m_odometry.GetEstimatedPosition();
    // frc::SmartDashboard::PutNumber( "Vision X", visionPose.X().value() );
    // frc::SmartDashboard::PutNumber( "Vision Y", visionPose.Y().value() );
    // frc::SmartDashboard::PutNumber( "Vision Theta", visionPose.Rotation().Degrees().value() );
    // frc::SmartDashboard::PutNumber( "Estimated X", estm_pose.X().value() );
    // frc::SmartDashboard::PutNumber( "Estimated Y", estm_pose.Y().value() );
    // frc::SmartDashboard::PutNumber( "Estimated Theta", estm_pose.Rotation().Degrees().value() );

//    frc::SmartDashboard::PutNumber( "Pitch", m_gyro.GetPitch() );
//    frc::SmartDashboard::PutNumber( "Roll", m_gyro.GetRoll() );

    m_field.SetRobotPose( estm_pose );
    m_field.GetObject( "Vision Pose" )->SetPose( visionPose );
}

// Resets the gyro to an angle
void Drivetrain::ResetGyro( units::degree_t angle ) {
    m_gyro.SetYaw( angle.value() );
}

// Returns the current pitch in degrees
// Front down on the robot is positive pitch
double Drivetrain::GetPitch( void ) {
    return m_gyro.GetPitch();
}

// Returns the pose2d of the robot
frc::Pose2d Drivetrain::GetPose( void ) {
    return m_odometry.GetEstimatedPosition();
}

// Resets the pose to a position
void Drivetrain::ResetPose( frc::Pose2d position ) {
        m_gyro.SetYaw(position.Rotation().Degrees().value());
        m_odometry.ResetPosition(
            frc::Rotation2d{units::degree_t{ m_gyro.GetYaw() }  },
            {
                m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                m_backLeft.GetPosition(), m_backRight.GetPosition() 
            },
            position
        );
}

void Drivetrain::AverageVisionPose( frc::Pose2d visionPose, units::second_t timestamp ) {

    if( !m_averagingPose ) {
        m_avgVisionPose = frc::Pose2d { 0_m, 0_m, 0_deg };
        m_averagingPose = true;
        m_avgIteration = 0;
        m_lastVisionTS = 0_s;
    }

    if( frc::Timer::GetFPGATimestamp() - m_lastVisionTS < m_averagingDelay ) return;

    m_avgVisionPose = frc::Pose2d{ visionPose.X() + m_avgVisionPose.X(),
                                    visionPose.Y() + m_avgVisionPose.Y(),
                                    visionPose.Rotation().Degrees() + m_avgVisionPose.Rotation().Degrees() };
    fmt::print( "Averaging in pose [{}] ({:.6}, {:.6}, {:.6}), sum({:.6}, {:.6}, {:.6}),  T[{:.6}]\n", m_avgIteration, 
                visionPose.X(), visionPose.Y(),  visionPose.Rotation().Degrees(),
                m_avgVisionPose.X(), m_avgVisionPose.Y(),  m_avgVisionPose.Rotation().Degrees(),
                timestamp );
    ++m_avgIteration;
    m_lastVisionTS = frc::Timer::GetFPGATimestamp();

    if( m_avgIteration == 10 ) {
        m_avgVisionPose = frc::Pose2d{ m_avgVisionPose.X() / m_avgIteration,
                                       m_avgVisionPose.Y() / m_avgIteration,
                                       m_avgVisionPose.Rotation().Degrees() / m_avgIteration };

  // TODO: The calculation of the gyro operator offset seems overly complicated.
  //       This does work but it is ugly and confusing.

        if ( m_avgVisionPose.X() < 7.5_m ) {
            fmt::print( "Blue Alliance\n");
            m_gyro_operator_offset = 360_deg - units::degree_t{ m_gyro.GetYaw() } - m_avgVisionPose.Rotation().Degrees();
        } else {
            fmt::print( "Red Alliance\n");
            m_gyro_operator_offset = 180_deg - units::degree_t{ m_gyro.GetYaw() } - m_avgVisionPose.Rotation().Degrees();
        }
        fmt::print( "Gyro Reading = {}\n", units::degree_t{ m_gyro.GetYaw() } );
        fmt::print( "Field Angle = {}\n", m_avgVisionPose.Rotation().Degrees() );
        fmt::print( "Gyro offset = {}\n", m_gyro_operator_offset );

        m_odometry.ResetPosition( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
        {
            m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
            m_backLeft.GetPosition(), m_backRight.GetPosition() 
        }, m_avgVisionPose );
        m_noValidPose = false;
        m_averagingPose = false;

        fmt::print( "Set vision pose average ({:.6}, {:.6}, {:.6}) [{}]!\n", 
                    m_avgVisionPose.X().value(), m_avgVisionPose.Y().value(),
                    m_avgVisionPose.Rotation().Degrees().value(), m_avgIteration );
    }
}


void Drivetrain::DrivetrainSetup() {
    m_frontLeft.ModuleSetup();
    m_frontRight.ModuleSetup();
    m_backLeft.ModuleSetup();
    m_backRight.ModuleSetup();
}

void Drivetrain::DrivetrainTest() {
    m_frontLeft.ModuleTest( "Front Left" );
    m_frontRight.ModuleTest( "Front Right" );
    m_backLeft.ModuleTest( "Back Left" );
    m_backRight.ModuleTest( "Back Right" );

    wpi::array opStates = { m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(), m_backRight.GetState() };
    frc::ChassisSpeeds speed = m_kinematics.ToChassisSpeeds(opStates);
    frc::SmartDashboard::PutNumber("robot vx", speed.vx.value() );
    frc::SmartDashboard::PutNumber("robot vy", speed.vy.value() );
    frc::SmartDashboard::PutNumber("robot Omega", speed.omega.value() );
}

void Drivetrain::PoseToNetworkTables() {
    frc::SmartDashboard::PutNumber( "Robot Pose X", m_odometry.GetEstimatedPosition().X().value() );
    frc::SmartDashboard::PutNumber( "Robot Pose Y", m_odometry.GetEstimatedPosition().Y().value() );
    frc::SmartDashboard::PutNumber( "Robot Pose Theta", m_odometry.GetEstimatedPosition().Rotation().Degrees().value() );
}