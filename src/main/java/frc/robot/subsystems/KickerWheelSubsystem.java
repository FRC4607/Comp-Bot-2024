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

public class KickerWheelSubsystem extends SubsystemBase {
    private final CANSparkMax m_kicker;
    private final RelativeEncoder m_encoder;
    private final DoubleLogEntry m_velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "kicker/velocity");

    /** Creates a new KickerWheelSubsystem. */
    public KickerWheelSubsystem() {
        m_kicker = new CANSparkMax(54, MotorType.kBrushless);
        m_encoder = m_kicker.getEncoder();
        m_kicker.restoreFactoryDefaults();
    }

    public void setKicker(double speed) {
        m_kicker.set(speed);
    }

    @Override
    public void periodic() {
        m_velocityLog.append(m_encoder.getVelocity());
    }
}
