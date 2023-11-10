#pragma once

#include <numbers>
#include <vector>

#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>

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

    constexpr int kWristMotorID = 15;

    constexpr int kFrontLeftAbsoluteEncoderID = 1;
    constexpr int kFrontRightAbsoluteEncoderID = 0;
    constexpr int kBackLeftAbsoluteEncoderID = 3;
    constexpr int kBackRightAbsoluteEncoderID = 2;
    constexpr int kArmEncoderID = 4;
    constexpr int kWristEncoderID = 5;

    constexpr int kPigeonIMUID = 13;

    constexpr int kGrabberSolenoidForwardChannel = 1;
    constexpr int kGrabberSolenoidReverseChannel = 0;
    constexpr int kBrakeSolenoidForwardChannel = 3;
    constexpr int kBrakeSolenoidReverseChannel = 4;

    constexpr int kLEDPWMID = 8;
}

namespace physical {
    // Max drive speed of Mk4 swerve modules * a scalar value
    constexpr units::meters_per_second_t kMaxDriveSpeed = 19.0_fps * 1.0;

    // The max speed of the turn motors
    constexpr auto kMaxTurnSpeed =  5_rad_per_s;

    // Old: Gear ratio of the drive motors. 6.86 rotations of the drive motor is one rotation of the wheel.
    // Mark IV L4 gear ratio 
    // https://www.swervedrivespecialties.com/products/mk3-to-mk4-upgrade-kit
    // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    constexpr double kDriveGearRatio = 5.14;

    // Compound unit for the meter per revolution constant.
    using meters_per_rev = units::compound_unit<units::meters, units::inverse<units::turns>>;
    typedef units::unit_t<meters_per_rev> meters_per_rev_t;

    // The number of meters traveled per rotation of the drive motor
    // wheel circumference / gear ratio
    constexpr meters_per_rev_t kDriveMetersPerRotation = std::numbers::pi * 4.0_in / (kDriveGearRatio *  1_tr );

    // Gear ratio of the arm
    constexpr double kOldArmGearRatio = 12.0 / 58.0 * 18.0 / 58.0 * 15.0 / 26.0;
    // constexpr double kArmGearRatio = 8.0 / 60.0 * 18.0 / 58.0 * 12.0 / 26.0;

    constexpr double kArmGearRatio = 12.0 / 58.0 * 16.0 / 60.0 * 12.0 / 26.0;

    // Conversion factor for falcon motors
    constexpr double tics_per_100ms_to_deg_per_s = 3600.0 / 2048.0;

    // Gear ratio of the turn motors. 12.8 rotations of the turning motor is one rotation of the swerve module.
    constexpr double kTurnGearRatio = 12.8;

    // Gear ratio between the absolute encoder for the wrist and the wrist
    constexpr double kWristEncoderGearRatio = 24.0 / 34.0;

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
    constexpr double kArmAbsoluteOffset = 0.495;

    constexpr units::degree_t kWristAbsoluteOffset = -53.7_deg;

    // IsFinished condition for arm
    constexpr units::degree_t kArmAngleError = 3_deg;

    constexpr units::inch_t kLimelightXAxisOffset = 325.61_in;
    constexpr units::inch_t kLimelightYAxisOffset = 157.8_in;

    constexpr units::inch_t kPlaceDistance = 11_in;

    constexpr units::degree_t kArmLowerPlaceHeight = -60_deg;
    constexpr units::degree_t kWristLowerPlaceHeight = 20_deg;
    constexpr double kLowerDelayProportion = 0.0;

    constexpr units::degree_t kArmConeMidPlaceHeight = 50_deg;
    constexpr units::degree_t kWristConeMidPlaceHeight = -165_deg;
    constexpr units::degree_t kArmCubeMidPlaceHeight = -30_deg;
    constexpr units::degree_t kWristCubeMidPlaceHeight = 0_deg;
    
    constexpr units::degree_t kArmSubstationMidPlaceHeight = 63_deg;
    constexpr units::degree_t kWristSubstationMidPlaceHeight = -175_deg;
    constexpr double kMidDelayProportion = 0.3; 

    constexpr units::degree_t kArmUpperPlaceHeight = 30_deg;
    constexpr units::degree_t kWristUpperPlaceHeight = -90_deg;
    constexpr double kUpperDelayProportion = 0.5;
}

namespace pidf {
    // PID values for turn motor of swerve modules
    constexpr double kTurnP = 0.006;
    constexpr double kTurnI = 0;
    constexpr double kTurnD = 0;

    constexpr double kTurnS = 0.0;
    constexpr double kTurnV = 0.01;
    constexpr double kTurnA = 0.0;

    // PID values for drive motor of swerve modules
//    constexpr double kDriveP = 0.0001;
    constexpr double kDriveP = 0.0001;
    constexpr double kDriveI = 0;
    constexpr double kDriveD = 0.0001;
    //    constexpr double kDriveFF = 0.00017;
    constexpr double kDriveFF = 0.00017;

    // P value for auto balance on charge station
 //   constexpr double kGyroBalanceP = 0.0075;
    constexpr double kGyroBalanceP = 0.03;

    //constexpr double kXTargetP = 0.005;
    //constexpr double kYTargetP = 0.1;
    //constexpr double kOmegaTargetP = 0.001;

    // PID values for wrist mechanism
    constexpr double kWristG = 0.3;
    constexpr double kWristS = 0.0;
    constexpr double kWristV = 1.5;
    constexpr double kWristA = 0.06;

    constexpr double kWristP = 0.004;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.0005;

    // PID values for arm mechanism
    // Before gear ratio change
    // constexpr double kArmG = 1.1;
    // constexpr double kArmGWrist = 0.4;
    // constexpr double kArmS = 0.0;
    // constexpr double kArmV = 0.35;

    // constexpr double kArmP = 0.0035;
    // constexpr double kArmI = 0.0;
    // constexpr double kArmD = 0.0002;

    // After gear ratio change
    constexpr double kArmG = 1.1 * physical::kArmGearRatio / physical::kOldArmGearRatio;
    constexpr double kArmGWrist = 0.4 * physical::kArmGearRatio / physical::kOldArmGearRatio;
    constexpr double kArmS = 0.0;
    constexpr double kArmV = 0.6;
    constexpr double kArmA = 0.0;

    constexpr double kArmP = 0.003;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0002 * physical::kOldArmGearRatio / physical::kArmGearRatio;

    
}

