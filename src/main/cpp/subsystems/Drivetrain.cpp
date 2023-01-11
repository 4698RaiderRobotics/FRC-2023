#include "subsystems/Drivetrain.h"

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

    wpi::array opStates = { m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(), m_backRight.GetState() };
    // Displays the SwerveModules current position
    swerve_display.SetState( opStates );

    // Updates the odometry of the robot given the SwerveModules' states
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(), m_backRight.GetPosition() 
    });
}

// Drives a path given a trajectory state
void Drivetrain::DriveTrajectory( frc::Trajectory::State trajectoryState ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetPose(), trajectoryState, trajectoryState.pose.Rotation().Degrees() );

    Drive( adjustedSpeeds );
}

// Returns the pose2d of the robot
frc::Pose2d Drivetrain::GetPose( void ) {
    return m_odometry.GetPose();
}

// Resets the gyro to an angle
void Drivetrain::ResetGyro( int angle ) {
    m_gyro.SetYaw( angle );
}

// Resets the pose to a position
void Drivetrain::ResetPose( frc::Translation2d position ) {
    m_odometry.ResetPosition(
        frc::Rotation2d{   units::degree_t{ m_gyro.GetYaw() }  },
        {
            m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
            m_backLeft.GetPosition(), m_backRight.GetPosition() 
        },
        frc::Pose2d{ position.X(), position.Y(), units::degree_t{ m_gyro.GetYaw() } }
    );
}