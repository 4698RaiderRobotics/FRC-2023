#pragma once 

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Limelight.h"
class VisionPose : public frc2::CommandHelper<frc2::CommandBase, VisionPose> {
    public:
        explicit VisionPose ( Limelight* limelight);
        void FixPose() override;
    private:
        Limelight* m_limelight;

};