// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/AddressableLED.h>
#include "Constants.h"

#include <array>

class LEDs : public frc2::SubsystemBase {
 public:
  LEDs();
  void Rainbow();
  void SetAll(int R, int G, int B);
  void SetAll(frc::Color color);
  void Chase(frc::Color color, int pixels);
  void Linear_Pulse(frc::Color color, units::time::second_t cycle);
  void Sinusoidal_Pulse(frc::Color color, units::time::second_t cycle);
  void Breath_Pulse(frc::Color color, units::second_t cycle);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr int kLength = 110;
  frc::AddressableLED m_led{deviceIDs::kLEDPWMID};

  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;
  int firstPixelHue = 0;
  int firstPixel = 0;
};
