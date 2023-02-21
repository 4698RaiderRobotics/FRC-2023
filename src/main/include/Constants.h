#pragma once

#include <numbers>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>

namespace pidf {
    constexpr double kTurnP = 0.005;
    constexpr double kTurnI = 0;
    constexpr double kTurnD = 0;

    constexpr double kDriveP = 0.00006;
    constexpr double kDriveI = 0.000001;
    constexpr double kDriveD = 0;
    constexpr double kDriveFF = 0.000015;

    constexpr double kGyroBalanceP = 0.005;

    constexpr double kXTargetP = 0.05;
    constexpr double kYTargetP = 0.5;
    constexpr double kOmegaTargetP = 0.01;
}

namespace deviceIDs {
    const int kFrontLeftTurnMotorID = 1;
    const int kFrontLeftDriveMotorID = 2;
    const int kBackLeftTurnMotorID = 3;
    const int kBackLeftDriveMotorID = 4;
    const int kBackRightTurnMotorID = 5;
    const int kBackRightDriveMotorID = 6;
    const int kFrontRightTurnMotorID = 7;
    const int kFrontRightDriveMotorID = 8;

    const int kFrontLeftAbsoluteEncoderID = 0;
    const int kFrontRightAbsoluteEncoderID = 1;
    const int kBackLeftAbsoluteEncoderID = 2;
    const int kBackRightAbsoluteEncoderID = 3;

    const int kPigeonIMUID = 13;
}

namespace physical {
    // Max drive speed of Mk3 swerve modules * a scalar value
    constexpr units::meters_per_second_t kMaxDriveSpeed = 14.4_fps * 0.5;

    // The max speed of the turn motors
    constexpr auto kMaxTurnSpeed =  5_rad_per_s;

    // Gear ratio of the drive motors. 6.86 rotations of the drive motor is one rotation of the wheel.
    constexpr double kDriveGearRatio = 6.86;

    // Compound unit for the meter per revolution constant.
    using meters_per_rev = units::compound_unit<units::meters, units::inverse<units::turns>>;
    typedef units::unit_t<meters_per_rev> meters_per_rev_t;

    // The number of meters traveled per rotation of the drive motor
    // wheel circumference / gear ratio
    constexpr meters_per_rev_t kDriveMetersPerRotation = std::numbers::pi * 4_in / (kDriveGearRatio *  1_tr );

    // Gear ratio of the turn motors. 12.8 rotations of the turning motor is one rotation of the swerve module.
    constexpr double kTurnGearRatio = 12.8;

    // The width of the drive base from the center of one module to another adjacent one.
    constexpr units::meter_t kDriveBaseWidth = 22.5_in;

    constexpr units::meter_t kDriveBaseLength = 22.5_in;

    constexpr double kFrontLeftAbsoluteOffset = 0.1244;
    constexpr double kFrontRightAbsoluteOffset = 0.3902;
    constexpr double kBackLeftAbsoluteOffset = 0.7013; 
    constexpr double kBackRightAbsoluteOffset = 0.5409;
}