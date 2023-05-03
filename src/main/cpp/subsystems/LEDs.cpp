// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDs.h"

LEDs::LEDs()
{
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void LEDs::Rainbow()
{
    // For every pixel
    for (int i = 0; i < kLength; i++)
    {
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
}
void LEDs::SetAll(int r, int g, int b) {
    SetAll(frc::Color(r, g, b));
}
void LEDs::SetAll(frc::Color colors) {
    for (auto pixel : m_ledBuffer)
        pixel.SetLED(colors);

}
void LEDs::Chase(frc::Color color, int tailLength) {
    for (int i = 0; i < kLength; i++) {
        int distance = abs(firstPixel - i);
        double modifier = ((tailLength + 1) - distance) / tailLength;
        modifier > 0 ? modifier : 0;
        auto pixelColor = frc::Color(
            color.red * modifier,
            color.green * modifier,
            color.blue * modifier
        );
        m_ledBuffer[i].SetLED(pixelColor);
    }
    // After we hit pixel kLength, wrap around to the first pixel.
    firstPixel = (firstPixel + 1) % kLength;
}

// This method will be called once per scheduler run
void LEDs::Periodic()
{
    m_led.SetData(m_ledBuffer);
}
