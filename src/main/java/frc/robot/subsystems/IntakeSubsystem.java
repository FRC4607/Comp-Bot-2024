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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/**
 * Subsystem for the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final TalonFXStandardSignalLogger m_log;
    // private final CANSparkFlex m_agitatorMotor;
    // private final SparkPIDController m_agitatorPid;
    // private final RelativeEncoder m_agitatorEncoder;

    private final VelocityTorqueCurrentFOC m_vel = new VelocityTorqueCurrentFOC(0);

    private final DigitalInput m_input;

    private double m_intakePowerCoefficient;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.kRollerGearRatio;
        config.MotorOutput.Inverted = IntakeConstants.kRollerInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = Calibrations.IntakeCalibrations.kRollerP;
        config.Slot0.kD = Calibrations.IntakeCalibrations.kRollerD;
        config.Slot0.kS = Calibrations.IntakeCalibrations.kRollerS;
        config.TorqueCurrent.PeakForwardTorqueCurrent = Calibrations.IntakeCalibrations.kRollerMaxCurrent;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -Calibrations.IntakeCalibrations.kRollerMaxCurrent;
        m_motor = new TalonFX(Constants.IntakeConstants.kRollerCANID, "kachow");
        m_motor.getConfigurator().apply(config);
        m_log = new TalonFXStandardSignalLogger(m_motor, "/intake");
        // m_agitatorMotor = new CANSparkFlex(IntakeConstants.kAgitatorCANID,
        // MotorType.kBrushless);
        // m_agitatorMotor.restoreFactoryDefaults();
        // m_agitatorMotor.setInverted(IntakeConstants.kAgitatorInverted);
        // m_agitatorMotor.setSmartCurrentLimit(60, 20, 3000);
        // m_agitatorEncoder = m_agitatorMotor.getEncoder();
        // m_agitatorEncoder.setVelocityConversionFactor(
        // (1.0 / 60.0) * // RPM -> RPS
        // (1.0 / IntakeConstants.kAgitatorGearRatio) * // Account for gearing
        // (Math.PI * IntakeConstants.kAgitatorDiameter) // RPS -> MM/S
        // );
        // CANSparkUtil.ConfigPIDCANSpark(IntakeCalibrations.kAgitatorP, 0, 0, 0,
        // m_agitatorMotor);
        // m_agitatorPid = m_agitatorMotor.getPIDController();

        m_input = new DigitalInput(0);

        m_intakePowerCoefficient = 1.0;
    }

    /**
     * Sets the open loop power of the intake motor.
     * 
     * @param power The open loop output of the motor [-1, 1]
     */
    public void setOpenLoopOutput(double power) {
        m_motor.set(power);
        // m_agitatorMotor.set(power * 0.5);
    }

    /**
     * Sets the setpoint (mm/s) for the intake to use with PID control.
     * 
     * @param newIntakeRMPSetpoint The new setpoint (mm/s) which updates the old
     *                             one.
     */
    public void setIntakeSetpoint(double newIntakeSetpoint) {
        m_motor.setControl(
                m_vel.withVelocity(newIntakeSetpoint / (Math.PI * Constants.IntakeConstants.kRollerDiameter)));
        // m_agitatorPid.setReference(newIntakeSetpoint, ControlType.kVelocity);
    }

    /**
     * Sets the max power to be used by the intake motor.
     * 
     * @param newIntakePowerCoefficient The new value which will be multiplied by
     *                                  the amp limits of the motor.
     */
    public void setIntakePower(double newIntakePowerCoefficient) {

        m_intakePowerCoefficient = newIntakePowerCoefficient;
    }

    /**
     * Returns the roller's velocity in mm/s.
     * 
     * @return The roller's velocity in mm/s.
     */
    public double intakeMmPerS() {
        return m_log.m_velocity.getValueAsDouble() * (Math.PI * Constants.IntakeConstants.kRollerDiameter);
    }

    /**
     * Returns true whenever the intake is intaking a note.
     * 
     * @return Whether there is a note currently in the intake.
     */
    public boolean hasNote() {
        return !m_input.get();
    }

    /**
     * Returns the agitator's velocity in mm/s.
     * 
     * @return The agitator's velocity in mm/s.
     */
    // public double agitatorMmPerS() {
    // return m_agitatorEncoder.getVelocity();
    // }

    public void periodic() {
        SmartDashboard.putBoolean("Beam Break", m_input.get());
    }
}
