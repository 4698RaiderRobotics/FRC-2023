#include "commands/GyroBalance.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/base.h>
using namespace units::literals;
GyroBalance::GyroBalance( Drivetrain* drive )
    : m_drive{ drive } {
    AddRequirements( { drive } );
}

void GyroBalance::Execute( ) {
    currentPitch = m_drive->GetPitch();
    currentRoll = m_drive->GetRoll();
    errorX = -currentPitch; 
    errorY = -currentRoll;
    //drivePower = units::meters_per_second(pidf::kGyroBalanceP * error);
    drivePowerX = units::meters_per_second_t(pidf::kGyroBalanceP * errorX.value());
    drivePowerY = units::meters_per_second_t(pidf::kGyroBalanceP * errorY.value());
    
    speeds.vx = drivePowerX;
    speeds.vy = drivePowerY;


    m_drive->Drive( speeds, false );
    
}

bool GyroBalance::IsFinished() {
    return ((units::math::abs( errorX) < 10_deg) && (units::math::abs( errorY) < 10_deg));
}