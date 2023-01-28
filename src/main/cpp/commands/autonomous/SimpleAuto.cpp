#include "commands/autonomous/SimpleAuto.h"
#include <iostream>

SimpleAuto::SimpleAuto( Drivetrain *drive ) {
    std::cout << "Simple Auto \n";
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "SimpleAuto.wpilib.json";
    m_simpleAutoTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

    const frc::Pose2d startPose{ 2.028_m, 4.668_m, frc::Rotation2d{ 0_deg } };
    const frc::Pose2d endPose{ 10.841_m, 2.258_m, frc::Rotation2d{ 0_deg } };

    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{ 6.052_m, 5.483_m },      
      frc::Translation2d{ 6.483_m, 1.475_m },
      frc::Translation2d{ 9.021_m, 5.962_m }
    };

    frc::TrajectoryConfig config{ 7_fps, 5_fps_sq };

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, interiorWaypoints, endPose, config);
    
    AddCommands( FollowTrajectory( drive, m_simpleAutoTrajectory) );
}