// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle = new CANdle(Constants.LEDConstants.kCANdleID, "kachow");

  private ShooterSubsystem m_shooter;
  private double m_pastVelocity;

  public LEDSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If the shooter drops by a critical value (100 rpm), LEDs turn red.
    // if (m_pastVelocity - m_shooter.outerShooterRPM() > 100) {
    // m_candle.setLEDs(255, 0, 0, 0, 1, m_ledCount);
    // }

    // m_pastVelocity = m_shooter.outerShooterRPM();
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGBW;
    config.brightnessScalar = 0.25;
    m_candle.configAllSettings(config);
  }

  /**
   * Method to set the LED color with RGBW values.
   * 
   * @param r red (0-255)
   * @param g green (0-255)
   * @param b Blue (0-255)
   * @param w white, if applicable to LEDs (0-255)
   */
  public void setLED(int r, int g, int b, int w) {
  }
}
