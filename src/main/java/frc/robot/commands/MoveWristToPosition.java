// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/**
 * A command that moves the wrist to a given position and waits for the wrist to
 * be within a given tolerance before finishing.
 */
public class MoveWristToPosition extends Command {
    private final DoubleSupplier m_position;
    private final double m_tol;
    private final WristSubsystem m_subsystem;
    private final boolean m_ignoreArm;

    private int m_debounce;

    /**
     * Creates a new MoveWristToPosition command.
     * 
     * @param position  The position to move to in degrees. 0deg points along +X,
     *                  90deg points along +Z.
     * @param tol       The tolerance to wait for before ending in degrees.
     * @param subsystem A reference to the
     *                  {@link frc.robot.subsystems.WristSubsystem} object.
     */
    public MoveWristToPosition(DoubleSupplier position, double tol, WristSubsystem subsystem, boolean ignoreArm) {
        m_position = position;
        m_tol = tol;
        m_subsystem = subsystem;
        m_ignoreArm = ignoreArm;
        addRequirements(m_subsystem);
    }

    public MoveWristToPosition(DoubleSupplier position, double tol, WristSubsystem subsystem) {
        this(position, tol, subsystem, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setIgnoreArm(m_ignoreArm);
        m_subsystem.setWristSetpoint(m_position);
        m_debounce = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(Math.IEEEremainder(m_ignoreArm ? m_subsystem.getRawWristPosition() : m_subsystem.getWristPosition(), 360.0) - m_position.getAsDouble()) < m_tol) {
            m_debounce++;
        } else {
            m_debounce = 0;
        }
        return m_debounce >= 4;
    }
}
