#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <cameraserver/CameraServer.h>

#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"

#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"

void Robot::RobotInit() {
  // Starts recording to data log
  frc::DataLogManager::Start();
  // Record both DS control and joystick data
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  // Log in Telelop:
//  frc::CameraServer::StartAutomaticCapture();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
ccc * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  // TODO: This call completely disabled to robot.  It would not respond
  //       to joystick input.  No motors would work.  Need to DEBUG
//    m_container.TestDataUpdate();

}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  std::cout << "auto init \n";
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  m_container.TeleopDataSetup();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  m_container.TeleopDataUpdate();
}

void Robot::TestInit() {
  m_container.TestDataSetup();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
  m_container.TestDataUpdate();
}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
