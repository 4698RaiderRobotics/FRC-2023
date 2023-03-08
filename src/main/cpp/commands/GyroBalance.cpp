#include "commands/GyroBalance.h"

GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );
}

void GyroBalance::Execute( ) {
    currentAngle = -m_drive->GetPitch();
    frc::SmartDashboard::PutNumber( "Pitch", currentAngle );

    error = 0 - currentAngle;

    drivePower = pidf::kGyroBalanceP * error * physical::kMaxDriveSpeed;

    speeds.vx = drivePower;

    m_drive->Drive( speeds, false );
    
}

bool GyroBalance::IsFinished() {
    return false;
}