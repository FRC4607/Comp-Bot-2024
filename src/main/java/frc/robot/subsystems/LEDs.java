// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDs extends SubsystemBase {
 private final CANdle m_candle = new CANdle(Constants.LEDConstants.CANdleID, "kachow");
 private final int LedCount = Constants.LEDConstants.LedCount;
  private int m_r = 0;
  private int m_g = 255;
  private int m_b = 0;
  private int m_w = 0;
  public LEDs() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setLED(int r, int g, int b, int w){
 m_candle.setLEDs(m_r,m_g,m_b,m_w,1,LedCount);




  }
}
