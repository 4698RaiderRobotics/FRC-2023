#include "commands/autonomous/SimpleAuto.h"

SimpleAuto::SimpleAuto( Drivetrain *drive ) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "pathplanner" / "generatedJSON" / "New Path.wpilib.json";
    m_simpleAutoTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

    AddCommands( FollowTrajectory( drive, m_simpleAutoTrajectory) );
}