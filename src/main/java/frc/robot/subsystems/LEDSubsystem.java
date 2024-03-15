// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(Constants.LEDConstants.kCANdleID, "kachow");

    private static enum LEDSubsystemState {
        DISABLED,
        NEUTRAL,
        INTAKE,
        SHOOT_NOT_READY,
        SHOOT_READY,
        AMP
    }

    private Alliance m_alliance = null;

    private static LEDSubsystemState m_currentState = LEDSubsystemState.DISABLED;
    private static LEDSubsystemState m_pastState = null;

    private final StrobeAnimation m_noAlliance = new StrobeAnimation(255, 0, 255, 0, 1, Constants.LEDConstants.kRGBCount);

    private final SingleFadeAnimation m_redDisabled = new SingleFadeAnimation(255, 0, 0, 0, 0.2, Constants.LEDConstants.kRGBCount);
    private final SingleFadeAnimation m_blueDisabled = new SingleFadeAnimation(0, 0, 255, 0, 0.2, Constants.LEDConstants.kRGBCount);

    private final LarsonAnimation m_intake = new LarsonAnimation(255, 165, 0, 0, 0.25, Constants.LEDConstants.kRGBCount, LarsonAnimation.BounceMode.Back, 3);

    private final TwinkleOffAnimation m_shootNotReady = new TwinkleOffAnimation(255, 0, 0, 0, 1, Constants.LEDConstants.kRGBCount, TwinkleOffAnimation.TwinkleOffPercent.Percent64);
    private final StrobeAnimation m_shootReady = new StrobeAnimation(0, 255, 0, 0, 1, Constants.LEDConstants.kRGBCount);

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // The chips we use seem to be RGB
        config.brightnessScalar = 0.25; // Avoid drawing too much current
        m_candle.configAllSettings(config);
    }

    @Override
    public void periodic() {
        boolean colorUpdate = false;
        if (m_alliance == null) {
            Optional<Alliance> value = DriverStation.getAlliance();
            if (value.isPresent()) {
                m_alliance = value.get();
                colorUpdate = true;
            }
        }
        if ((m_currentState != m_pastState) || colorUpdate) {
            switch (m_currentState) {
                case DISABLED:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.animate(m_blueDisabled);
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.animate(m_redDisabled);
                    } else {
                        m_candle.animate(m_noAlliance);
                    }
                    break;
                case NEUTRAL:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.setLEDs(0, 0, 255, 0, 0, Constants.LEDConstants.kRGBCount);
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.setLEDs(255, 0, 0, 0, 0, Constants.LEDConstants.kRGBCount);
                    } else {
                        m_candle.animate(m_noAlliance);
                    }
                    break;
                case INTAKE:
                    m_candle.animate(m_intake);
                    break;
                case SHOOT_NOT_READY:
                    m_candle.animate(m_shootNotReady);
                    break;
                case SHOOT_READY:
                    m_candle.animate(m_shootReady);
                    break;
                case AMP:
                    m_candle.setLEDs(255, 165, 0, 0, 0, Constants.LEDConstants.kRGBCount);
                    break;
            }
        }
        m_pastState = m_currentState;
    }

    public static void setDisabled() {
        m_currentState = LEDSubsystemState.DISABLED;
    }

    public static void setNeutral() {
        m_currentState = LEDSubsystemState.NEUTRAL;
    }

    public static void setIntake() {
        m_currentState = LEDSubsystemState.INTAKE;
    }

    public static void setShootNotReady() {
        m_currentState = LEDSubsystemState.SHOOT_NOT_READY;
    }

    public static void setShootReady() {
        m_currentState = LEDSubsystemState.SHOOT_READY;
    }
    
    public static void setAmp() {
        m_currentState = LEDSubsystemState.AMP;
    }
}
