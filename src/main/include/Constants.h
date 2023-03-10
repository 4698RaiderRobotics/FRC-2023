#pragma once

#include <numbers>
#include <vector>

#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>

namespace pidf {
    // PID values for turn motor of swerve modules
    constexpr double kTurnP = 0.0095;
    constexpr double kTurnI = 0;
    constexpr double kTurnD = 0;

    // PID values for drive motor of swerve modules
    constexpr double kDriveP = 0.0001;
    constexpr double kDriveI = 0;
    constexpr double kDriveD = 0.001;
    constexpr double kDriveFF = 0.0002;

    // P value for auto balance on charge station
    constexpr double kGyroBalanceP = 0.004;

    //constexpr double kXTargetP = 0.005;
    //constexpr double kYTargetP = 0.1;
    //constexpr double kOmegaTargetP = 0.001;
}

namespace deviceIDs {
    constexpr int kFrontLeftTurnMotorID = 7;
    constexpr int kFrontLeftDriveMotorID = 8;
    constexpr int kBackLeftTurnMotorID = 5;
    constexpr int kBackLeftDriveMotorID = 6;
    constexpr int kBackRightTurnMotorID = 3;
    constexpr int kBackRightDriveMotorID = 4;
    constexpr int kFrontRightTurnMotorID = 1;
    constexpr int kFrontRightDriveMotorID = 2;

    constexpr int kLeftArmMotorID = 11;
    constexpr int kRightArmMotorID = 12;

    constexpr int kFrontLeftAbsoluteEncoderID = 1;
    constexpr int kFrontRightAbsoluteEncoderID = 0;
    constexpr int kBackLeftAbsoluteEncoderID = 3;
    constexpr int kBackRightAbsoluteEncoderID = 2;
    constexpr int kArmEncoderID = 4;

    constexpr int kPigeonIMUID = 13;

    constexpr int kGrabberSolenoidForwardChannel = 1;
    constexpr int kGrabberSolenoidReverseChannel = 0;
    constexpr int kBrakeSolenoidForwardChannel = 3;
    constexpr int kBrakeSolenoidReverseChannel = 4;
}

namespace physical {
    // Max drive speed of Mk3 swerve modules * a scalar value
    constexpr units::meters_per_second_t kMaxDriveSpeed = 14.4_fps * 1;

    // The max speed of the turn motors
    constexpr auto kMaxTurnSpeed =  5_rad_per_s;

    // Gear ratio of the drive motors. 6.86 rotations of the drive motor is one rotation of the wheel.
    constexpr double kDriveGearRatio = 6.86;

    // Compound unit for the meter per revolution constant.
    using meters_per_rev = units::compound_unit<units::meters, units::inverse<units::turns>>;
    typedef units::unit_t<meters_per_rev> meters_per_rev_t;

    // The number of meters traveled per rotation of the drive motor
    // wheel circumference / gear ratio
    constexpr meters_per_rev_t kDriveMetersPerRotation = std::numbers::pi * 4.0_in / (kDriveGearRatio *  1_tr );

    // Gear ratio of the arm
    constexpr double kArmGearRatio = 12.0 / 58.0 * 18.0 / 58.0 * 15.0 / 26.0;

    // Conversion factor for falcon motors
    constexpr double tics_per_100ms_to_deg_per_s = 3600.0 / 2048.0;

    // Gear ratio of the turn motors. 12.8 rotations of the turning motor is one rotation of the swerve module.
    constexpr double kTurnGearRatio = 12.8;

    // The width of the drive base from the center of one module to another adjacent one.
    constexpr units::meter_t kDriveBaseWidth = 23.25_in;

    // Length of the drive base
    constexpr units::meter_t kDriveBaseLength = 22.5_in;

    // Absolute encoder offset for the swerve modules
    constexpr double kFrontLeftAbsoluteOffset = 0.1372 + 0.5;
    constexpr double kFrontRightAbsoluteOffset = 0.3699 + 0.5;
    constexpr double kBackLeftAbsoluteOffset = 0.7882 - 0.5;
    constexpr double kBackRightAbsoluteOffset = 0.4485 + 0.5;

    // Absolute encoder offset for the arm encoder
    constexpr double kArmAbsoluteOffset = 0.5802;

    // IsFinished condition for arm
    constexpr units::degree_t kArmAngleError = 3_deg;

    constexpr units::inch_t kLimelightXAxisOffset = 325.61_in;
    constexpr units::inch_t kLimelightYAxisOffset = 157.8_in;

    constexpr units::inch_t kPlaceDistance = 17_in;
}