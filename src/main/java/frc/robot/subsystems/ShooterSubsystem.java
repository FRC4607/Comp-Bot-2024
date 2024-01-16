// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.rev.CANSparkUtil;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex m_upper;
  private final CANSparkFlex m_lower;

  private final SparkPIDController m_uPID;
  private final SparkPIDController m_lPID;

  private final RelativeEncoder m_uEnc;
  private final RelativeEncoder m_lEnc;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_upper = new CANSparkFlex(56, MotorType.kBrushless);
    m_lower = new CANSparkFlex(55, MotorType.kBrushless);
    
    m_lower.restoreFactoryDefaults();
    m_upper.restoreFactoryDefaults();

    m_upper.setInverted(true);

    CANSparkUtil.ConfigPIDCANSpark(0.0002, 0.0, 0.00005, 0.00015, m_lower);
    CANSparkUtil.ConfigPIDCANSpark(0.0002, 0.0, 0.00005, 0.00015, m_upper);

    m_uPID = m_upper.getPIDController();
    m_lPID = m_lower.getPIDController();

    m_uEnc = m_upper.getEncoder();
    m_lEnc = m_lower.getEncoder();
  }

  public void setSpeed(double speed) {
    //System.out.println(speed);
    m_uPID.setReference(speed, ControlType.kVelocity);
    m_lPID.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Avg Shooter Speed", (m_uEnc.getVelocity() + m_lEnc.getVelocity()) / 2);
  }
}
