
#include "commands/autonomous/SimpleAuto.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/ArmSet.h"
#include "commands/TestProfileMove.h"
#include "commands/OpenGrabber.h"
#include "commands/autonomous/FollowTrajectory.h"

SimpleAuto::SimpleAuto( Drivetrain *drive, ArmSubsystem *arm, GrabberSubsystem *grabber, 
                        units::degree_t angle ) {
  /*
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "SimpleAuto.wpilib.json";
    m_simpleAutoTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  */

      // The start is the rightmost cone grid next to AT#3 (for testing).
    const frc::Pose2d startPose{ drive->redAllianceGridPoints[8].X(), drive->redAllianceGridPoints[8].Y(), 180_deg };
      // The end is around the back of the charge station.
    const frc::Pose2d endPose{ 10.75_m, 3.38_m, 270_deg };

    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{ 11.452_m, 4.9_m }
    };

    frc::TrajectoryConfig config{ 1_mps, 1_mps_sq };

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, interiorWaypoints, endPose, config);
    
    AddCommands( 
      DriveToPoseCommand( drive, drive->redAllianceGridPoints[8] ),
      ArmSet( angle, arm ),
      TestProfileMove( 18_in, TestProfileMove::FORWARD, drive ),
      OpenGrabber( grabber ),
      TestProfileMove( -18_in, TestProfileMove::FORWARD, drive ),
      ArmSet( -90_deg, arm ),
      FollowTrajectory( drive, m_trajectory, 180_deg ) 
    );
}