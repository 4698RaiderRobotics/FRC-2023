#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"

class UpdateOdom : public frc2::CommandHelper<frc2::CommandBase, UpdateOdom> {
    public:    
        explicit UpdateOdom( Drivetrain *drive, Limelight *limelight);
        void Start();
    private:
        Drivetrain *m_drive;
        Limelight *m_limelight;
};