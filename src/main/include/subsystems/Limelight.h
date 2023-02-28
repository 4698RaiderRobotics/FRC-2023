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
#include "Constants.h"

class Limelight : public frc2::SubsystemBase {
    public:
        Limelight(void);
        
        frc::ChassisSpeeds TargetRobot_AT ( void );

        bool Finished( void );

        bool VisionPose( frc::Pose2d* );
        
        void SetPipeline( int pipelineId );
        
        void LimelightTest();

    private:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        // The X position of the target in the limelight's view
        double targetX;
        // The Y position of the target in the limelight's view
        double targetY;

        std::vector<double> camtran{};
        std::vector<double> botpose{};
        std::span<double> defaultValue{};
        double ll_Pose[3];
        frc::Pose2d l_Pose;
        units::degree_t rZ;
        frc::ChassisSpeeds t_speeds;

        double kXTargetP = 0.005;
        double kYTargetP = 0.1;
        double kOmegaTargetP = 0.001;
};