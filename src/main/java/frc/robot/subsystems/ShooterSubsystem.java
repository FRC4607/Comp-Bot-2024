// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ShooterCalibrations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.rev.CANSparkUtil;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex m_upper;
    private final CANSparkFlex m_lower;

    private final SparkPIDController m_uPID;
    private final SparkPIDController m_lPID;

    private final RelativeEncoder m_uEnc;
    private final RelativeEncoder m_lEnc;

    private final DoubleLogEntry m_uVelLog = new DoubleLogEntry(DataLogManager.getLog(), "shooter/upper_vel");
    private final DoubleLogEntry m_lVelLog = new DoubleLogEntry(DataLogManager.getLog(), "shooter/lower_vel");

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_upper = new CANSparkFlex(ShooterConstants.kUpperCANId, MotorType.kBrushless);
        m_lower = new CANSparkFlex(ShooterConstants.kLowerCANId, MotorType.kBrushless);

        m_lower.restoreFactoryDefaults();
        m_upper.restoreFactoryDefaults();

        m_upper.setInverted(true);
        m_lower.setInverted(false);

        CANSparkUtil.ConfigPIDCANSpark(ShooterCalibrations.kP, ShooterCalibrations.kI, ShooterCalibrations.kD, ShooterCalibrations.kFF, m_lower);
        CANSparkUtil.ConfigPIDCANSpark(ShooterCalibrations.kP, ShooterCalibrations.kI, ShooterCalibrations.kD, ShooterCalibrations.kFF, m_upper);

        m_uPID = m_upper.getPIDController();
        m_lPID = m_lower.getPIDController();

        m_uEnc = m_upper.getEncoder();
        m_lEnc = m_lower.getEncoder();
    }

    public void setSpeed(double speed) {
        // System.out.println(speed);
        m_uPID.setReference(speed, ControlType.kVelocity);
        m_lPID.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Avg Shooter Speed", (m_uEnc.getVelocity() + m_lEnc.getVelocity()) / 2);
        m_uVelLog.append(m_uEnc.getVelocity());
        m_lVelLog.append(m_lEnc.getVelocity());
    }
}
