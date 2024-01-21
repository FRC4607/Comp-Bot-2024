// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex m_motor;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_motor = new CANSparkFlex(IntakeConstants.kCANId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        // m_motor.setSmartCurrentLimit(60, 20, 3000);
    }

    public void setPower(double power) {
        m_motor.set(power);
    }
}
