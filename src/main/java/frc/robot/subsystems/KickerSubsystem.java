// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/**
 * Subsystem that controls the kicker.
 */
public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_kicker;

    private final TalonFXStandardSignalLogger m_log;
    private final DoubleLogEntry m_setpointVelocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/kicker/setpoint/velocity");
    private final DoubleLogEntry m_setpointPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "/kicker/setpoint/position");

    private final VelocityTorqueCurrentFOC m_velocity;
    private final MotionMagicTorqueCurrentFOC m_position;

    /** Creates a new KickerSubsystem. */
    public KickerSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = KickerConstants.kKickerGearRatio;
        config.MotorOutput.Inverted = KickerConstants.kInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = KickerCalibrations.kP_vel;
        config.Slot0.kD = KickerCalibrations.kD_vel;
        config.Slot0.kS = KickerCalibrations.kS_vel;
        config.Slot0.kV = KickerCalibrations.kV_vel;
        config.Slot1.kP = KickerCalibrations.kP_pos;
        config.Slot1.kD = KickerCalibrations.kD_pos;
        config.Slot1.kS = KickerCalibrations.kS_pos;
        config.Slot1.kV = KickerCalibrations.kV_pos;
        config.MotionMagic.MotionMagicAcceleration = Calibrations.KickerCalibrations.kMotionMagicMaxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = Calibrations.KickerCalibrations.kMotionMagicMaxVelocity;
        m_kicker = new TalonFX(KickerConstants.kCANID, "kachow");
        m_kicker.getConfigurator().apply(config);

        m_log = new TalonFXStandardSignalLogger(m_kicker, "/kicker");

        m_velocity = new VelocityTorqueCurrentFOC(0);
        m_velocity.Slot = 0;
        m_position = new MotionMagicTorqueCurrentFOC(0);
        m_position.Slot = 1;
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
    public void setKickerSetpointVelocity(double velocity) {
        m_velocity.Velocity = velocity / (Math.PI * Constants.KickerConstants.kKickerDiameter);
        m_setpointVelocityLog.append(m_velocity.Velocity);
        m_kicker.setControl(m_velocity);
    }

    /**
     * Sets the closed loop position of the kicker wheels in rotations.
     * 
     * @param speed The velocity to set the kicker wheels to in roations.
     */
    public void setKickerSetpointPosition(double position) {
        //System.out.println("Set position setpoint to " + position);
        m_position.Position = position;
        m_setpointPositionLog.append(m_position.Position);
        m_kicker.setControl(m_position);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Kicker RPM", (m_velocity.Velocity / (Math.PI * Constants.KickerConstants.kKickerDiameter)) * 60);
        m_log.log();
    }

    public double getIntakePosition() {
        return m_log.m_pos.getValueAsDouble();
    }
}
