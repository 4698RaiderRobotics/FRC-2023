
#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"

Drivetrain::Drivetrain( Limelight *ll ) : m_limelight{ll} {
//    if( frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ) {
        ResetGyro( 180_deg );
//    } else {
//        ResetGyro( 0_deg );
//   }
    frc::SmartDashboard::PutData("Field", &m_field);
}

// Drives with joystick inputs
// This takes -1 to 1 inputs
void Drivetrain::Drive( double xSpeed, double ySpeed, double omegaSpeed, bool fieldRelative ) {
    auto x = xSpeed * physical::kMaxDriveSpeed;
    auto y = ySpeed * physical::kMaxDriveSpeed;
    auto omega = omegaSpeed * physical::kMaxTurnSpeed;

    frc::ChassisSpeeds speeds{ x, y , omega };

    Drive( speeds, fieldRelative );
}

void Drivetrain::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    //m_field.SetRobotPose(m_odometry.GetPose());
    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    auto states = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
                    speeds.vx, speeds.vy, speeds.omega, frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } } ) :
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
void Drivetrain::DriveTrajectory( frc::Trajectory::State trajectoryState ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetEstimatedPosition(), trajectoryState, trajectoryState.pose.Rotation().Degrees() );

    Drive( adjustedSpeeds );
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
        if( m_noValidPose ) {
            if( !m_averagingPose ) {
                m_avgVisionPose = frc::Pose2d { 0_m, 0_m, 0_deg };
                m_averagingPose = true;
                m_avgIteration = 1;
            }

            m_avgVisionPose = frc::Pose2d{ visionPose.X() + m_avgVisionPose.X(),
                                           visionPose.Y() + m_avgVisionPose.Y(),
                                           visionPose.Rotation().Degrees() + m_avgVisionPose.Rotation().Degrees() };
            fmt::print( "Averaging vision pose ({}, {}, {}) [{}]\n",  m_avgVisionPose.X().value(),
                        m_avgVisionPose.Y().value(),  m_avgVisionPose.Rotation().Degrees().value(), m_avgIteration );

            if( m_avgIteration == 10 ) {
                m_odometry.ResetPosition( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
                {
                    m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                    m_backLeft.GetPosition(), m_backRight.GetPosition() 
                }, m_avgVisionPose / m_avgIteration );
                m_noValidPose = false;
                m_averagingPose = false;
                fmt::print( "Set vision pose average ({}, {}, {}) [{}]!\n", 
                            m_avgVisionPose.X().value() / m_avgIteration,
                            m_avgVisionPose.Y().value() / m_avgIteration,
                            m_avgVisionPose.Rotation().Degrees().value() / m_avgIteration,
                            m_avgIteration );
            }
            ++m_avgIteration;
        } else {
            if( m_odometry.GetEstimatedPosition().Translation().Distance( visionPose.Translation() ) < 1_m ) {
                // Only update if the vision pose is within 1m of the current pose
                m_odometry.AddVisionMeasurement( visionPose, timestamp );
            }
        }
    }

    frc::Pose2d estm_pose = m_odometry.GetEstimatedPosition();
    frc::SmartDashboard::PutNumber( "Vision X", visionPose.X().value() );
    frc::SmartDashboard::PutNumber( "Vision Y", visionPose.Y().value() );
    frc::SmartDashboard::PutNumber( "Vision Theta", visionPose.Rotation().Degrees().value() );
    frc::SmartDashboard::PutNumber( "Estimated X", estm_pose.X().value() );
    frc::SmartDashboard::PutNumber( "Estimated Y", estm_pose.Y().value() );
    frc::SmartDashboard::PutNumber( "Estimated Theta", estm_pose.Rotation().Degrees().value() );

    m_field.SetRobotPose( estm_pose );
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