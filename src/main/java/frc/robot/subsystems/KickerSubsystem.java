// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.KickerCalibrations;
import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/**
 * Subsystem that controls the kicker.
 */
public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_kicker;

    private final TalonFXStandardSignalLogger m_log;
    private final DoubleLogEntry m_setpointLog = new DoubleLogEntry(DataLogManager.getLog(), "/kicker/setpoint");

    private final VelocityTorqueCurrentFOC m_pid;

    /** Creates a new KickerSubsystem. */
    public KickerSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = KickerConstants.kKickerGearRatio;
        config.MotorOutput.Inverted = KickerConstants.kInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = KickerCalibrations.kP;
        config.Slot0.kD = KickerCalibrations.kD;
        config.Slot0.kS = KickerCalibrations.kS;
        config.Slot0.kV = KickerCalibrations.kV;
        m_kicker = new TalonFX(KickerConstants.kCANID, "kachow");
        m_kicker.getConfigurator().apply(config);

        m_log = new TalonFXStandardSignalLogger(m_kicker, "/kicker");

        m_pid = new VelocityTorqueCurrentFOC(0);
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
     * Sets the closed loop velocity of the kicker wheels in mm/s.
     * 
     * @param speed The velocity to set the kicker wheels to in mm/s.
     */
    public void setKickerSetpoint(double velocity) {
        m_pid.Velocity = velocity / (Math.PI * Constants.KickerConstants.kKickerDiameter);
        m_setpointLog.append(m_pid.Velocity);
        m_kicker.setControl(m_pid);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Kicker RPM", (m_pid.Velocity / (Math.PI * Constants.KickerConstants.kKickerDiameter)) * 60);
        m_log.log();
    }
}
