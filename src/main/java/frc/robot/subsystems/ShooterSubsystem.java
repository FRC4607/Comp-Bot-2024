// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ShooterCalibrations;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/**
 * Subsystem for the shooter on the robot.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_outer;
    private final TalonFX m_inner;

    private final TalonFXStandardSignalLogger m_outerLog;
    private final TalonFXStandardSignalLogger m_innerLog;

    private final DoubleLogEntry m_setpointLog = new DoubleLogEntry(DataLogManager.getLog(), "/shooter/setpoint");

    private final VelocityTorqueCurrentFOC m_req;
    private final NeutralOut m_neutral = new NeutralOut();
    private boolean m_useNeutral = false;
    private final Follower m_follow;

    private DoubleSupplier m_speedSupplier;

    private double m_shooterPowerCoefficient;
    private double m_pastVelocity;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_outer = new TalonFX(ShooterConstants.kOuterCANID, "kachow");
        m_inner = new TalonFX(ShooterConstants.kInnerCANID, "kachow");

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Slot0.kS = ShooterCalibrations.kS;
        config.Slot0.kP = ShooterCalibrations.kP;

        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterCalibrations.kMaxCurrent;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -ShooterCalibrations.kMaxCurrent;

        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertOuter ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_outer.getConfigurator().apply(config);
        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertOuter ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        m_inner.getConfigurator().apply(config);

        m_follow = new Follower(ShooterConstants.kOuterCANID, true);

        m_outerLog = new TalonFXStandardSignalLogger(m_outer, "/shooter/outer");
        m_innerLog = new TalonFXStandardSignalLogger(m_inner, "/shooter/inner");

        m_req = new VelocityTorqueCurrentFOC(0);

        m_speedSupplier = () -> 0.0;

        m_shooterPowerCoefficient = 1.0;
    }

    @Override
    public void periodic() {
        m_outerLog.log();
        m_innerLog.log();
        m_setpointLog.append(m_speedSupplier.getAsDouble());
        if (m_useNeutral) {
            m_outer.setControl(m_neutral);
        } else {
            m_outer.setControl(m_req.withVelocity(m_speedSupplier.getAsDouble() / 60.0));
        }
        m_inner.setControl(m_follow);
        SmartDashboard.putNumber("Target Shooter RPM", m_speedSupplier.getAsDouble());
    }

    /**
     * Sets the RPM setpoint of the shooter to be used with PID control.
     * 
     * @param newShooterRPMsetpoint The new setpoint (RPM) which updates the old
     *                              one.
     */
    public void setShooterRPMSetpoint(DoubleSupplier newShooterRPMSetpoint) {
        m_useNeutral = false;
        m_speedSupplier = newShooterRPMSetpoint;
    }

    public void setNetural() {
        m_useNeutral = true;
        m_speedSupplier = () -> 0.0;
    }

    /**
     * Sets the max power that the shooter can use.
     * 
     * @param newShooterPowerCoefficient The new value which will be multiplied by
     *                                   the amp limits of each motor.
     */
    public void setShooterPower(double newShooterPowerCoefficient) {
        m_shooterPowerCoefficient = newShooterPowerCoefficient;
    }

    /**
     * Returns the current RPM of the outer shooter.
     * 
     * @return The current RPM of the outer shooter.
     */
    public double outerShooterRPM() {
        return m_outerLog.m_velocity.getValueAsDouble() * 60.0;
    }

    /**
     * Returns the current RPM of the inner shooter.
     * 
     * @return The current RPM of the inner shooter.
     */
    public double innerShooterRPM() {
        return m_innerLog.m_velocity.getValueAsDouble() * 60.0;
    }
    

}
