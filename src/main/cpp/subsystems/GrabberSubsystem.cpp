// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/LEDs.h"
#include <frc/Timer.h>

GrabberSubsystem::GrabberSubsystem(ArmSubsystem *arm, LEDs *leds) : frc2::SubsystemBase(), m_arm{arm}, m_leds{leds}
{
    m_intake.RestoreFactoryDefaults();
    m_intake.SetSmartCurrentLimit(40);
}

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic()
{

    double stallSpeed = m_enc.GetVelocity();
    auto elapsedTime = frc::Timer::GetFPGATimestamp() - m_startTime;
    if ((m_hasCone || m_hasCube) && m_isEjecting) {
        // Currently Ejecting
        if (elapsedTime > 1.0_s) {
            // Finsished Ejecting
            m_hasCone = false;
            m_hasCube = false;
            m_intake.Set(0.0);
            m_leds->Rainbow();
        }
    } else if (m_loadingCone || m_loadingCube) {
            // Loading a game piece
        if ((elapsedTime > 0.5_s) && (fabs(stallSpeed) < m_target_stall_speed)) {
            fmt::print("GrabberSubsystem::Periodic stopped with speed of {}\n", stallSpeed);
            if (m_loadingCone) {
                m_hasCone = true;
                m_loadingCone = false;
                m_intake.Set(-0.05);
                m_leds->SetAll(frc::Color::kYellow);
            } else {
                m_hasCube = true;
                m_loadingCube = false;
                m_intake.Set(0.05);
                m_leds->SetAll(frc::Color::kPurple);
            }
            //           m_intake.Set( 0.0 );
        } else if (elapsedTime > 5_s) {
            // Timed out without getting a game piece
            m_loadingCone = false;
            m_loadingCube = false;
            m_intake.Set(0.0);
        }
    }
    // fmt::print( "Intake current: {}\n", current );

}

// Speed of rollers is value from -1 to 1
void GrabberSubsystem::Spin(double speed)
{
    m_spin_speed = speed;
    m_intake.Set(m_spin_speed);
}
units::ampere_t GrabberSubsystem::GetCurrent()
{
    return units::ampere_t{m_intake.GetOutputCurrent()};
}

void GrabberSubsystem::HandleCone(void)
{
    if (!m_hasCone && m_hasCube)
    {
        // If we already have a cube and we click the Cone Button
        this->HandleCube();
        return;
    }

    m_startTime = frc::Timer::GetFPGATimestamp();

    if (m_hasCone)
    {
        // If we already have a cone shoot it out.
        m_isEjecting = true;
        m_intake.Set(m_cone_shoot_speed);
    }
    else
    {
        // Suck in the cone
        m_leds->Chase(frc::Color::kYellow, 20);
        m_loadingCone = true;
        m_loadingCube = false;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cone_max_amps;
        m_intake.Set(m_cone_intake_speed);
    }
}

void GrabberSubsystem::HandleCube(void)
{
    if (!m_hasCube && m_hasCone)
    {
        // If we already have a Cone and we hit the Cube button, Spit out the Cone Instead of intaking a cube
        this->HandleCone();
        return;
    }

    m_startTime = frc::Timer::GetFPGATimestamp();

    if (m_hasCube)
    {
        // Edjecting Cube Logic
        m_isEjecting = true;
        if (m_arm->GetArmAngle() < 10_deg)
        {
            //            m_leds->Sinusoidal_Pulse(frc::Color::kBlue, 2_s);
            m_intake.Set(m_cube_shoot_slow_speed);
        }
        else
        {
            //            m_leds->Sinusoidal_Pulse(frc::Color::kBlue, 1_s);
            m_intake.Set(m_cube_shoot_fast_speed);
        }
    }
    else
    {
        m_leds->Chase(frc::Color::kPurple, 20);
        // Intake Cube Logic
        m_loadingCone = false;
        m_loadingCube = true;
        m_hasCone = false;
        m_hasCube = false;
        m_isEjecting = false;
        m_target_amps = m_cube_max_amps;
        m_intake.Set(m_cube_intake_speed);
    }
}

bool GrabberSubsystem::HasCube() {
    return m_hasCube;
}

bool GrabberSubsystem::HasCone() {
    return m_hasCone;
}

void GrabberSubsystem::GrabberTest()
{
    // frc::SmartDashboard::PutData( &m_grab );
    // frc::SmartDashboard::PutNumber( "Roller Speed", m_roller.GetMotorOutputPercent() );
    // fmt::print( "GrabberTest" );
    frc::SmartDashboard::PutNumber("Intake Speed", m_intake.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intake.GetOutputCurrent());
}