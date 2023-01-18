#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <vector>
#include <frc/smartdashboard/Field2d.h>

#include "frc/smartdashboard/SmartDashboard.h"
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

        frc::Pose2d VisionPose( void );
        
        void SetPipeline( int pipelineId );

    private:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        frc::Field2d m_field;

        // The X position of the target in the limelight's view
        double targetX;
        // The Y position of the target in the limelight's view
        double targetY;

        // P value for omega targeting
        double kOmegaP = 0.01;
        // P value for x targeting
        double kxP = 0.05;
        // P value for y targeting
        double kyP = 0.5;
        std::vector<double> camtran{};
        std::vector<double> botpose{};
        std::vector<double> defaultValue{};
        units::degree_t rZ;
        frc::ChassisSpeeds t_speeds;
};