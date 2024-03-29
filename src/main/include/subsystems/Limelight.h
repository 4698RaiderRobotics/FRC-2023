#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <vector>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/SubsystemBase.h>

#include <wpi/DataLog.h>

#include "Constants.h"

class Limelight : public frc2::SubsystemBase {
    public:
        Limelight(void);

        void Periodic( void ) override;
        
        bool getFieldAprilTagPose( frc::Pose2d &, units::second_t & );

        bool TargetRobot_AT_Unused( frc::Pose2d& );

        bool VisionPose_Unused( frc::Pose2d* );
        
        void SetPipeline( int pipelineId );
        
        void LimelightTest();
    
    private:
        bool haveValidAprilTag( void );

    private:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        // The X position of the target in the limelight's view
        double targetX;
        // The Y position of the target in the limelight's view
        double targetY;

        std::vector<double> botpose{};
        std::span<double> defaultValue{};
        double ll_Pose[3];
        frc::Pose2d l_Pose;
        units::degree_t rZ;
        frc::Pose2d t_speeds;
        wpi::log::DoubleArrayLogEntry m_poseLogEntry;
};