// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Calibrations.ShooterCalibrations;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem for the shooter on the robot.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_upper;
    private final TalonFX m_lower;

    private final StatusSignal<Double> m_upperVel;
    private final StatusSignal<Double> m_lowerVel;

    private final VelocityTorqueCurrentFOC m_req;
    private final NeutralOut m_neutral;

    // private final DoubleLogEntry m_uVelLog = new DoubleLogEntry(DataLogManager.getLog(), "shooter/upper_vel");
    // private final DoubleLogEntry m_lVelLog = new DoubleLogEntry(DataLogManager.getLog(), "shooter/lower_vel");

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_upper = new TalonFX(ShooterConstants.kUpperCANId);
        m_lower = new TalonFX(ShooterConstants.kLowerCANId);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Slot0.kS = ShooterCalibrations.kS;
        config.Slot0.kP = ShooterCalibrations.kP;

        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertUpper ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_upper.getConfigurator().apply(config);
        config.MotorOutput.Inverted = Constants.ShooterConstants.kInvertLower ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_lower.getConfigurator().apply(config);

        m_upperVel = m_upper.getVelocity();
        m_lowerVel = m_lower.getVelocity();

        m_req = new VelocityTorqueCurrentFOC(0);

        m_neutral = new NeutralOut();
    }

    /**
     * Sets the speed of the shooter.
     * 
     * @param speed The speed to target in RPM.
     */
    public void setSpeed(double speed) {
        // System.out.println(speed);
        if (Double.compare(speed, 0.0) == 0) {
            m_upper.setControl(m_neutral);
            m_lower.setControl(m_neutral);
        } else {
            m_upper.setControl(m_req.withVelocity(speed / 60.0));
            m_lower.setControl(m_req.withVelocity(speed / 60.0));
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_upperVel, m_lowerVel);
        SmartDashboard.putNumber("Avg Shooter Speed",
                (m_upperVel.getValueAsDouble() + m_lowerVel.getValueAsDouble()) / 2);
        // m_uVelLog.append(m_upperVel.getValueAsDouble()); // This function was causing
        // a ton of loop overruns at one point, so these lines are commented out, at
        // least for now.
        // m_lVelLog.append(m_lowerVel.getValueAsDouble());
    }
}
