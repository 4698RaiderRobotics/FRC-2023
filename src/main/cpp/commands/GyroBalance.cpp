#include "commands/GyroBalance.h"

GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );
}

void GyroBalance::Execute( ) {
    currentAngle = m_drive->GetPitch();

    error = 0 - currentAngle;

    drivePower = units::meters_per_second_t{pidf::kGyroBalanceP * error};

    speeds.vx = drivePower;

    m_drive->Drive( speeds, false );
    
}
