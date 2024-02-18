// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/**
 * A command that moves the wrist to a given position and waits for the wrist to
 * be within a given tolerance before finishing.
 */
public class MoveWristToPosition extends Command {
    private final double m_position;
    private final double m_tol;
    private final WristSubsystem m_subsystem;

    /**
     * Creates a new MoveWristToPosition command.
     * 
     * @param position  The position to move to in degrees. 0deg points along +X,
     *                  90deg points along +Z.
     * @param tol       The tolerance to wait for before ending in degrees.
     * @param subsystem A reference to the
     *                  {@link frc.robot.subsystems.WristSubsystem} object.
     */
    public MoveWristToPosition(double position, double tol, WristSubsystem subsystem) {
        m_position = position;
        m_tol = tol;
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setWristSetpoint(m_position);
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
        return Math.abs(m_subsystem.getWristPosition() - m_position) < m_tol;
    }
}
