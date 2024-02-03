// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsystem for the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex m_motor;
    private final SparkPIDController m_pid;
    private final RelativeEncoder m_encoder;

    private double m_intakePowerCoefficient;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_motor = new CANSparkFlex(IntakeConstants.kCANId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(IntakeConstants.kInverted);
        m_motor.setSmartCurrentLimit(60, 20, 3000);
        m_pid = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_intakePowerCoefficient = 1.0;
    }

    /**
     * Sets the open loop power of the intake motor.
     * 
     * @param power The open loop output of the motor [-1, 1]
     */
    public void setOpenLoopOutput(double power) {
        m_motor.set(power);
    }

    /**
     * Sets the setpoint (RPM) for the arm to use with PID control.
     * 
     * @param newIntakeRMPSetpoint The new setpoint (RPM) which updates the old one.
     */
    public void setIntakeRPMSetpoint(double newIntakeRMPSetpoint) {
        m_pid.setReference(newIntakeRMPSetpoint, ControlType.kVelocity);
    }

    /**
     * Sets the max power to be used by the intake motors.
     * 
     * @param newIntakePowerCoefficient The new value which will be multiplied by
     *                                  the amp limits of each motor.
     */
    public void setIntakePower(double newIntakePowerCoefficient) {

        m_intakePowerCoefficient = newIntakePowerCoefficient;
    }

    /**
     * Returns the intake velocity in RPMs.
     * 
     * @return the intake velocity in RPMs.
     */
    public double intakeRPM() {
        return m_encoder.getVelocity();
    }
}
