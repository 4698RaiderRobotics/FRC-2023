#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#include <units/time.h>
#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"
// #include "SwerveModuleDisplay.h"

class Limelight;

class Drivetrain : public frc2::SubsystemBase {
  public:
    Drivetrain( Limelight * );

    void ArcadeDrive( double xSpeed, double ySpeed, double omegaSpeed, bool operatorRelative = true );

    void Drive( frc::ChassisSpeeds speeds, bool fieldRelative = true );

    void DriveTrajectory( frc::Trajectory::State trajectoryState, const frc::Rotation2d &robotHeading );

    void StopDrive( );

    void Periodic() override;

    void ResetGyro( units::degree_t angle );

    double GetPitch( void );

    frc::Pose2d GetPose( void );

    void ResetPose( frc::Pose2d position );

    void DrivetrainSetup();

    void DrivetrainTest();

    void PoseToNetworkTables();

  private:
    void AverageVisionPose( frc::Pose2d visionPose, units::second_t timestamp );

    void LogSwerveStateArray( wpi::log::DoubleArrayLogEntry& m_logEntry, wpi::array<frc::SwerveModuleState, 4U> states );
    
    Limelight *m_limelight;
    bool m_noValidPose = true;
    bool m_averagingPose = false;
    int m_avgIteration = 0;
    units::second_t m_averagingDelay = 100_ms;
    units::second_t m_lastVisionTS = 0_s;
    frc::Pose2d m_avgVisionPose;
    units::degree_t m_gyro_operator_offset = 180_deg;

//    SwerveStatusDisplay swerve_display{ "Swerve Drive", "Robot Wheel Status" };

    SwerveModule m_frontLeft{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                            deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset };
    SwerveModule m_frontRight{ deviceIDs::kFrontRightTurnMotorID, deviceIDs::kFrontRightDriveMotorID, 
                            deviceIDs::kFrontRightAbsoluteEncoderID, physical::kFrontRightAbsoluteOffset };
    SwerveModule m_backLeft{ deviceIDs::kBackLeftTurnMotorID, deviceIDs::kBackLeftDriveMotorID, 
                            deviceIDs::kBackLeftAbsoluteEncoderID, physical::kBackLeftAbsoluteOffset };
    SwerveModule m_backRight{ deviceIDs::kBackRightTurnMotorID, deviceIDs::kBackRightDriveMotorID, 
                            deviceIDs::kBackRightAbsoluteEncoderID, physical::kBackRightAbsoluteOffset };

    frc::Trajectory m_trajectory;
    ctre::phoenix::sensors::PigeonIMU m_gyro{deviceIDs::kPigeonIMUID};

    wpi::array<frc::SwerveModuleState, 4U> m_desiredStates{ wpi::empty_array };
    wpi::array<frc::SwerveModuleState, 4U> m_actualStates{ wpi::empty_array };

    frc::Translation2d m_frontLeftLocation{ +( physical::kDriveBaseLength / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_frontRightLocation{ +( physical::kDriveBaseLength / 2 ), -( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_backLeftLocation{ -( physical::kDriveBaseLength / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_backRightLocation{ -( physical::kDriveBaseLength / 2 ), -( physical::kDriveBaseWidth / 2 ) };

    frc::SwerveDriveKinematics<4> m_kinematics{ m_frontLeftLocation, m_frontRightLocation, 
                                              m_backLeftLocation,m_backRightLocation};
    frc::SwerveDrivePoseEstimator<4> m_odometry{m_kinematics, frc::Rotation2d{ 0_deg },
      {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_backLeft.GetPosition(), m_backRight.GetPosition()
      },
 //     frc::Pose2d{ 610.77_in, 42.19_in, 0_deg } 
      frc::Pose2d{0.0_in, 0.0_in, 0_deg } 
    };
    // Drive controller for driving a trajectory
    frc::HolonomicDriveController m_controller{ 
          frc2::PIDController{ 6, 0, 0 }, frc2::PIDController{ 6, 0, 0 },
          frc::ProfiledPIDController<units::radian> {
            10, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
              6.28_rad_per_s, 3.14_rad_per_s / 1_s}}};

  wpi::log::DoubleArrayLogEntry m_actualLogEntry;
  wpi::log::DoubleArrayLogEntry m_desiredLogEntry;
  wpi::log::DoubleArrayLogEntry m_poseLogEntry;

  public:
    frc::Field2d m_field;
    frc::Pose2d redAllianceGridPoints[9];
    frc::Pose2d blueAllianceGridPoints[9];
    frc::Trajectory twoPiecePrac;
    frc::Trajectory twoPiecePrac2;
};