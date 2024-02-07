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
    private final CANSparkFlex m_rollerMotor;
    private final SparkPIDController m_rollerPid;
    private final RelativeEncoder m_rollerEncoder;
    private final CANSparkFlex m_agitatorMotor;
    private final SparkPIDController m_agitatorPid;
    private final RelativeEncoder m_agitatorEncoder;

    private double m_intakePowerCoefficient;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_rollerMotor = new CANSparkFlex(IntakeConstants.kRollerCANId, MotorType.kBrushless);
        m_rollerMotor.restoreFactoryDefaults();
        m_rollerMotor.setInverted(IntakeConstants.kRollerInverted);
        m_rollerMotor.setSmartCurrentLimit(60, 20, 3000);
        m_rollerPid = m_rollerMotor.getPIDController();
        m_rollerEncoder = m_rollerMotor.getEncoder();
        m_agitatorMotor = new CANSparkFlex(IntakeConstants.kAgitatorCANId, MotorType.kBrushless);
        m_agitatorMotor.restoreFactoryDefaults();
        m_agitatorMotor.setInverted(IntakeConstants.kAgitatorInverted);
        m_agitatorMotor.setSmartCurrentLimit(60, 20, 3000);
        m_agitatorPid = m_agitatorMotor.getPIDController();
        m_agitatorEncoder = m_agitatorMotor.getEncoder();
        m_intakePowerCoefficient = 1.0;
    }

    /**
     * Sets the open loop power of the intake motor.
     * 
     * @param power The open loop output of the motor [-1, 1]
     */
    public void setOpenLoopOutput(double power) {
        m_rollerMotor.set(power);
        m_agitatorMotor.set(power);
    }

    /**
     * Sets the setpoint (RPM) for the arm to use with PID control.
     * 
     * @param newIntakeRMPSetpoint The new setpoint (RPM) which updates the old one.
     */
    public void setIntakeRPMSetpoint(double newIntakeRMPSetpoint) {
        m_rollerPid.setReference(newIntakeRMPSetpoint, ControlType.kVelocity);
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
        return m_rollerEncoder.getVelocity();
    }
}
