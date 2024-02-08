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
import frc.robot.Calibrations.IntakeCalibrations;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.rev.CANSparkUtil;

/**
 * Subsystem for the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex m_rollerMotor;
    private final SparkPIDController m_rollerPid;
    private final RelativeEncoder m_rollerEncoder;
    // private final CANSparkFlex m_agitatorMotor;
    // private final SparkPIDController m_agitatorPid;
    // private final RelativeEncoder m_agitatorEncoder;

    private double m_intakePowerCoefficient;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_rollerMotor = new CANSparkFlex(IntakeConstants.kRollerCANId, MotorType.kBrushless);
        m_rollerMotor.restoreFactoryDefaults();
        m_rollerMotor.setInverted(IntakeConstants.kRollerInverted);
        m_rollerMotor.setSmartCurrentLimit(60, 20, 3000);
        m_rollerEncoder = m_rollerMotor.getEncoder();
        m_rollerEncoder.setVelocityConversionFactor(
            (1.0 / 60.0) * // RPM -> RPS
            (1.0 / IntakeConstants.kRollerGearRatio) * // Account for gearing
            (Math.PI * IntakeConstants.kRollerDiameter) // RPS -> MM/S
        );
        CANSparkUtil.ConfigPIDCANSpark(IntakeCalibrations.kRollerP, 0, 0, IntakeCalibrations.kRollerFF, m_rollerMotor);
        m_rollerPid = m_rollerMotor.getPIDController();
        // m_agitatorMotor = new CANSparkFlex(IntakeConstants.kAgitatorCANId, MotorType.kBrushless);
        // m_agitatorMotor.restoreFactoryDefaults();
        // m_agitatorMotor.setInverted(IntakeConstants.kAgitatorInverted);
        // m_agitatorMotor.setSmartCurrentLimit(60, 20, 3000);
        // m_agitatorEncoder = m_agitatorMotor.getEncoder();
        // m_agitatorEncoder.setVelocityConversionFactor(
        //     (1.0 / 60.0) * // RPM -> RPS
        //     (1.0 / IntakeConstants.kAgitatorGearRatio) * // Account for gearing
        //     (Math.PI * IntakeConstants.kAgitatorDiameter) // RPS -> MM/S
        // );
        // CANSparkUtil.ConfigPIDCANSpark(IntakeCalibrations.kAgitatorP, 0, 0, 0, m_agitatorMotor);
        // m_agitatorPid = m_agitatorMotor.getPIDController();
        m_intakePowerCoefficient = 1.0;
    }

    /**
     * Sets the open loop power of the intake motor.
     * 
     * @param power The open loop output of the motor [-1, 1]
     */
    public void setOpenLoopOutput(double power) {
        m_rollerMotor.set(power);
        // m_agitatorMotor.set(power * 0.5);
    }

    /**
     * Sets the setpoint (mm/s) for the arm to use with PID control.
     * 
     * @param newIntakeRMPSetpoint The new setpoint (mm/s) which updates the old one.
     */
    public void setIntakeSetpoint(double newIntakeSetpoint) {
        m_rollerPid.setReference(newIntakeSetpoint, ControlType.kVelocity);
        // m_agitatorPid.setReference(newIntakeSetpoint, ControlType.kVelocity);
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
     * Returns the roller's velocity in mm/s.
     * 
     * @return The roller's velocity in mm/s.
     */
    public double intakeMmPerS() {
        return m_rollerEncoder.getVelocity();
    }

    /**
     * Returns the agitator's velocity in mm/s.
     * 
     * @return The agitator's velocity in mm/s.
     */
    public double agitatorMmPerS() {
        //return m_agitatorEncoder.getVelocity();
        return 0.0;
    }
}
