#include "commands/GyroBalance.h"

GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );

    frc::SmartDashboard::PutNumber("Gyro kGB", pidf::kGyroBalanceP);

}

// Calculates a speed for the motors based on what the gyro reads
void GyroBalance::Execute( ) {

    currentAngle = units::degree_t{ -m_drive->GetPitch() };
 
   if (direction && currentAngle < 0_deg) {
        direction = false;
        flip++;
    } else if (!direction && currentAngle > 0_deg) {
        direction = true;
        flip++;
    }
    
    // fmt::print("flip = {},", flip);
    // fmt::print("d = {},", direction);
    // fmt::print("angle = {},", currentAngle);

    error = 0_deg - currentAngle;

     double tkGB = frc::SmartDashboard::GetNumber("Gyro kGB", 0.0);
    units::meters_per_second_t driveSpeed = tkGB * error.value() * 1_mps / ( 1.0 + 0.2 * (flip-1));
 
 //   units::meters_per_second_t driveSpeed = pidf::kGyroBalanceP * error.value() * 1_mps;
 
    // frc::SmartDashboard::PutNumber("Gyro Percent Reduction", flip);
    // frc::SmartDashboard::PutNumber("Gyro Angle", currentAngle.value());
    // frc::SmartDashboard::PutNumber("Gyro Angle Error", error.value());
    // frc::SmartDashboard::PutNumber("Gyro Drive speed", driveSpeed.value());

    speeds.vx = driveSpeed;
    //speeds.vx = 0.5_mps;

    m_drive->Drive( speeds, false );
    
}

bool GyroBalance::IsFinished() {
    return false;
}