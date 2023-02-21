#include "subsystems/Drivetrain.h"
Drivetrain::Drivetrain() {
    ResetGyro( 0 );
    //frc::SmartDashboard::PutData("Field", &m_field);

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

    wpi::array opStates = { m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(), m_backRight.GetState() };
    // Displays the SwerveModules current position
    swerve_display.SetState( opStates );

    // Updates the odometry of the robot given the SwerveModules' states
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
}

// Drives a path given a trajectory state
void Drivetrain::DriveTrajectory( frc::Trajectory::State trajectoryState ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetPose(), trajectoryState, trajectoryState.pose.Rotation().Degrees() );

    Drive( adjustedSpeeds );
}

void Drivetrain::Periodic( ) {
    // Updates the odometry of the robot given the SwerveModules' states
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(), m_backRight.GetPosition() 
    });
}

// Resets the gyro to an angle
void Drivetrain::ResetGyro( int angle ) {
    m_gyro.SetYaw( angle );
}

// Returns the current pitch in degrees
// Front down on the robot is positive pitch
double Drivetrain::GetPitch( void ) {
    return m_gyro.GetPitch();
}

// Returns the pose2d of the robot
frc::Pose2d Drivetrain::GetPose( void ) {
    return m_odometry.GetPose();
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

void Drivetrain::DrivetrainTest() {
    m_frontLeft.ModuleTest( "Front Left" );
    m_frontRight.ModuleTest( "Front Right" );
    m_backLeft.ModuleTest( "Back Left" );
    m_backRight.ModuleTest( "Back Right" );
}