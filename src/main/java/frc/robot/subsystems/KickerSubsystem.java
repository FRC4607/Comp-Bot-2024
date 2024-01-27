// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerWheelConstants;

/**
 * Subsystem that controls the kicker.
 */
public class KickerSubsystem extends SubsystemBase {
    private final CANSparkMax m_kicker;
    private final RelativeEncoder m_encoder;
    private final DoubleLogEntry m_velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "kicker/velocity");

    /** Creates a new KickerSubsystem. */
    public KickerSubsystem() {
        m_kicker = new CANSparkMax(KickerWheelConstants.kCANId, MotorType.kBrushless);
        m_kicker.restoreFactoryDefaults();
        m_encoder = m_kicker.getEncoder();
    }

    /**
     * Sets the open loop speed of the kicker motor.
     * @param speed The open loop speed to set the kicker to, [-1, 1].
     */
    public void setKicker(double speed) {
        m_kicker.set(speed);
    }

    @Override
    public void periodic() {
        m_velocityLog.append(m_encoder.getVelocity());
    }
}
