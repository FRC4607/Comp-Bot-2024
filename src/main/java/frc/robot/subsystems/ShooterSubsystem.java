// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Calibrations.ShooterCalibrations;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem for the shooter on the robot.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_outer;
    private final TalonFX m_inner;

    private final StatusSignal<Double> m_outerVel;
    private final StatusSignal<Double> m_innerVel;

    private final VelocityTorqueCurrentFOC m_req;
    private final Follower m_follow;
    private final NeutralOut m_neutral;

    private double m_shooterPowerCoefficient;

    // private final DoubleLogEntry m_uVelLog = new
    // DoubleLogEntry(DataLogManager.getLog(), "shooter/upper_vel");
    // private final DoubleLogEntry m_lVelLog = new
    // DoubleLogEntry(DataLogManager.getLog(), "shooter/lower_vel");

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_outer = new TalonFX(ShooterConstants.kOuterCANId);
        m_inner = new TalonFX(ShooterConstants.kInnerCANId);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Slot0.kS = ShooterCalibrations.kS;
        config.Slot0.kP = ShooterCalibrations.kP;

        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertOuter ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_outer.getConfigurator().apply(config);
        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertOuter ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        m_inner.getConfigurator().apply(config);

        m_follow = new Follower(ShooterConstants.kOuterCANId, true);

        m_outerVel = m_outer.getVelocity();
        m_innerVel = m_inner.getVelocity();

        m_req = new VelocityTorqueCurrentFOC(0);

        m_neutral = new NeutralOut();

        m_shooterPowerCoefficient = 1.0;
    }

    @Override
    public void periodic() {
        m_inner.setControl(m_follow);
        BaseStatusSignal.refreshAll(m_outerVel, m_innerVel);
        // m_uVelLog.append(m_upperVel.getValueAsDouble()); // This function was causing
        // a ton of loop overruns at one point, so these lines are commented out, at
        // least for now.
        // m_lVelLog.append(m_lowerVel.getValueAsDouble());
    }

    /**
     * Sets the RPM setpoint of the shooter to be used with PID control.
     * 
     * @param newShooterRPMsetpoint The new setpoint (RPM) which updates the old
     *                              one.
     */
    public void setShooterRPMSetpoint(double newShooterRPMSetpoint) {
        if (Double.compare(newShooterRPMSetpoint, 0.0) == 0) {
            m_outer.setControl(m_neutral);
        } else {
            m_inner.setControl(m_req.withVelocity(newShooterRPMSetpoint / 60.0));
        }
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
        return m_outerVel.getValueAsDouble() * 60.0;
    }

    /**
     * Returns the current RPM of the outer shooter.
     * 
     * @return the current RPM of the outer hooter.
     */
    public double innerShooterRPM() {
        return m_innerVel.getValueAsDouble() * 60.0;
    }
}
