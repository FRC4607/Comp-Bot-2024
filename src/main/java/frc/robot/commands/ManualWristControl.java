// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command that allows setting the position of the wrist manually.
 */
public class ManualWristControl extends Command {
    private final DoubleSupplier m_value;

    private final WristSubsystem m_subsystem;

    private double targetAngle;

    /**
     * Creates a new ManualWristControl.
     * 
     * @param value     A function that supplies a value in the range of [-1, 1] to
     *                  be used to control the wrist.
     * @param subsystem A reference to the wrist subsystem.
     */
    public ManualWristControl(DoubleSupplier value, WristSubsystem subsystem) {
        m_value = value;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetAngle = m_subsystem.getPIDSetpoint();
        m_subsystem.isRunning = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        targetAngle += WristConstants.kWristManualDegreesPerSecond * m_value.getAsDouble() * 0.02;
        m_subsystem.setWristSetpoint(targetAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.isRunning = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
