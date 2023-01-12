#include "commands/GyroBalance.h"

GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );
}

void GyroBalance::Execute( ) {
    speeds.vx = -m_drive->GetPitch( ) * pidf::kGyroBalanceP * physical::kMaxDriveSpeed;

    m_drive->Drive( speeds, false );
}