#pragma once

#include <frc/kinematics/ChassisSpeeds.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>

#include "Constants.h"

class Limelight {
    public:
        frc::ChassisSpeeds TargetRobot( void );

        void SetPipeline( int pipelineId );

    private:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        // The X position of the target in the limelight's view
        double targetX;
        // The Y position of the target in the limelight's view
        double targetY;

        // P value for omega targeting
        double kOmegaP = 0.02;
        // P value for x targeting
        double kxP = 0.05;

        frc::ChassisSpeeds t_speeds;
};