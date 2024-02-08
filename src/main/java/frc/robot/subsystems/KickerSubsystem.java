// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.KickerCalibrations;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.rev.CANSparkUtil;

/**
 * Subsystem that controls the kicker.
 */
public class KickerSubsystem extends SubsystemBase {
    private final CANSparkMax m_kicker;
    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;
    private final DoubleLogEntry m_velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "kicker/velocity");

    /** Creates a new KickerSubsystem. */
    public KickerSubsystem() {
        m_kicker = new CANSparkMax(KickerConstants.kCANId, MotorType.kBrushless);
        m_kicker.restoreFactoryDefaults();
        m_kicker.setInverted(true);
        m_encoder = m_kicker.getEncoder();
        m_encoder.setVelocityConversionFactor(
            (1.0 / 60.0) * // RPM -> RPS
            (1.0 / KickerConstants.kKickerGearRatio) * // Account for gearing
            (Math.PI * KickerConstants.kKickerDiameter) // RPS -> MM/S
        );
        CANSparkUtil.ConfigPIDCANSpark(KickerCalibrations.kP, 0, KickerCalibrations.kD, KickerCalibrations.kFF, m_kicker);
        m_pid = m_kicker.getPIDController();
    }

    /**
     * Sets the open loop speed of the kicker motor.
     * 
     * @param speed The open loop speed to set the kicker to, [-1, 1].
     */
    public void setOpenLoop(double speed) {
        m_kicker.set(speed);
    }

    /**
     * Sets the closed loop speed of the kicker wheels in mm/s.
     * 
     * @param speed The speed to set the kicker wheels to in mm/s.
     */
    public void setKickerSetpoint(double speed) {
        m_pid.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        m_velocityLog.append(m_encoder.getVelocity());
    }
}
