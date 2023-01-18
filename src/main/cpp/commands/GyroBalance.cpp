#include "commands/GyroBalance.h"

GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );
}

void GyroBalance::Execute( ) {
    currentAngle = m_drive->GetPitch();

    error = 0 - currentAngle;

    drivePower = -fmin( pidf::kGyroBalanceP * error, 1);

    speeds.vx = drivePower * 1_mps;

    m_drive->Drive( speeds, false );
    
}

bool GyroBalance::IsFinished( ) {
    return abs( error ) < 1;
}