#include "commands/autonomous/SimpleAuto.h"

SimpleAuto::SimpleAuto( Drivetrain *drive ) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "SimpleAuto.wpilib.json";
    m_simpleAutoTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

    const frc::Pose2d startPose{ 0_ft, 0_ft, frc::Rotation2d{ 0_deg } };
    const frc::Pose2d endPose{ 8_ft, 0_ft, frc::Rotation2d{ 0_deg } };

    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{ 4_ft, 0_ft }
    };

    frc::TrajectoryConfig config{ 7_fps, 5_fps_sq };

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, interiorWaypoints, endPose, config);

    AddCommands( FollowTrajectory( drive, m_simpleAutoTrajectory) );
}