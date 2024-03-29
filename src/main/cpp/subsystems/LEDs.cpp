// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDs.h"
#include "frc/Timer.h"
#include <iostream>
LEDs::LEDs()
{
    m_led.SetLength(m_ledBuffer.size());
    m_led.SetData(m_ledBuffer);
    m_led.Start();

    m_currColor = frc::Color::kBlue;
    m_chaseTaillength = 10;
    m_cycleTime = 5_s;
}


void LEDs::SetAll(int r, int g, int b) {
    SetAll(frc::Color(r, g, b));
}
void LEDs::SetAll(frc::Color color) {
    /*for (auto pixel : m_ledBuffer)
        pixel.SetLED(colors);*/
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetLED(color);
    }
    m_currentLedCommand = constant;
}
void LEDs::Rainbow() {
    m_currentLedCommand = rainbow;
}
void LEDs::Chase(frc::Color color, int tailLength) {
    m_currColor = color;
    m_chaseTaillength = tailLength;
    m_currentLedCommand = chase;
}

void LEDs::Linear_Pulse(frc::Color color, units::second_t cycle) {
    m_currColor = color;
    m_cycleTime = cycle;
    m_currentLedCommand = lpulse;
}
void LEDs::Sinusoidal_Pulse(frc::Color color, units::second_t cycle) {
    m_currColor = color;
    m_cycleTime = cycle;
    m_currentLedCommand = spulse;
}
// Internal commands
void LEDs::f_chase(frc::Color color, int tailLength) {
    for (auto i = 0; i < kLength; i++) {
        int distance = std::abs(firstPixel - i);
        double modifier = ((tailLength + 1) - distance) / tailLength;
        modifier = modifier > 0 ? modifier : 0;
        auto pixelColor = frc::Color(
            color.red * modifier,
            color.green * modifier,
            color.blue * modifier
        );
        m_ledBuffer[i].SetLED(pixelColor);

    }
    // After we hit pixel kLength, wrap around to the first pixel.
    firstPixel = (firstPixel + 1) % kLength;
    //std::cout << "Chasing\n";
}

void LEDs::f_linear_pulse(frc::Color color, units::time::second_t cycle) {
    units::second_t time = frc::Timer::GetFPGATimestamp();
    double loop = fabs(remainder(time.value(), cycle.value()));
    double magnitude = (2 * loop) / cycle.value();
    auto pixelColor = frc::Color(
        color.red * magnitude,
        color.green * magnitude,
        color.red * magnitude
    );
    SetAll(pixelColor);
    //    std::cout << "l Pulsing\n";
}
void LEDs::f_sinusoidal_pulse(frc::Color color, units::time::second_t cycle) {
    units::second_t time = frc::Timer::GetFPGATimestamp();
    double loop = fabs(remainder(time.value(), cycle.value()));
    double t = (2 * loop) / cycle.value();
    // Clamp cos(t) to 0-1 and start at 0
    double magnitude = (cos(t + std::numbers::pi) + 1) / 2;
    auto pixelColor = frc::Color(
        color.red * magnitude,
        color.green * magnitude,
        color.red * magnitude
    );
    SetAll(pixelColor);
    //    std::cout << "s Pulsing\n";
}
void LEDs::f_rainbow() {
    // For every pixel
    for (size_t i = 0; i < m_ledBuffer.size(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
        // Set the value
        m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // Check bounds
    firstPixelHue %= 180;
    //    std::cout << "displaying Rainbow\n";
}
// This method will be called once per scheduler run
void LEDs::Periodic()
{
    m_led.SetData(m_ledBuffer);
    if (m_currentLedCommand == chase) {
        //Chase(frc::Color::kBlue, 10);
        f_chase(m_currColor, m_chaseTaillength);
    }
    else if (m_currentLedCommand == rainbow) {
        // Rainbow();
        f_rainbow();
    }
    else if (m_currentLedCommand == lpulse) {
        //Linear_Pulse(frc::Color::kBlue, 5_s);
        f_linear_pulse(m_currColor, m_cycleTime);
    }
    else if (m_currentLedCommand == spulse) {
        //Sinusoidal_Pulse(frc::Color::kBlue, 5_s);
        f_sinusoidal_pulse(m_currColor, m_cycleTime);
    }
}
