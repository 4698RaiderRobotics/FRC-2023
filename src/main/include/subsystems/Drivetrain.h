#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <units/time.h>
#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"
#include "SwerveModuleDisplay.h"


class Drivetrain : public frc2::SubsystemBase {
  public:
    Drivetrain( void );

    void Drive( double xSpeed, double ySpeed, double omegaSpeed, bool fieldRelative = true );

    void Drive( frc::ChassisSpeeds speeds, bool fieldRelative = true );

    void DriveTrajectory( frc::Trajectory::State trajectoryState );

    void ResetGyro( int angle );

    double GetPitch( void );

    frc::Pose2d GetPose( void );
    void ResetPose( frc::Pose2d position );

  private:
    frc::Field2d m_field;

    SwerveStatusDisplay swerve_display{ "Swerve Drive", "Robot Wheel Status" };

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

    frc::Translation2d m_frontLeftLocation{ +( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_frontRightLocation{ +( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_backLeftLocation{ -( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    frc::Translation2d m_backRightLocation{ -( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };

    frc::SwerveDriveKinematics<4> m_kinematics{ m_frontLeftLocation, m_frontRightLocation, 
                                              m_backLeftLocation,m_backRightLocation};
    frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, frc::Rotation2d{ 0_deg },
      {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_backLeft.GetPosition(), m_backRight.GetPosition()
      },
      frc::Pose2d{ 0_ft, 0_ft, 0_deg } 
    };
    // Drive controller for driving a trajectory
    frc::HolonomicDriveController m_controller{ 
          frc2::PIDController{ 1, 0, 0 }, frc2::PIDController{ 1, 0, 0 },
          frc::ProfiledPIDController<units::radian> {
            1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
              6.28_rad_per_s, 3.14_rad_per_s / 1_s}}};
};