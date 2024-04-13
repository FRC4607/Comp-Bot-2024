// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Sets the shooter to a given velocity and waits for it to be within a given
 * tolerance before ending.
 */
public class SetShooterSpeed extends Command {
    private final DoubleSupplier m_speed;
    private final double m_tol;
    private final ShooterSubsystem m_subsystem;

    /**
     * Creates a new SetShooterSpeed.
     * 
     * @param speedRPM  The velocity to set the shooter to in rotations per minute.
     * @param tol       The tolerance to wait for before ending the command in
     *                  rotations per minute.
     * @param subsystem A reference to the
     *                  {@link frc.robot.subsystems.ShooterSubsystem} object.
     */
    public SetShooterSpeed(DoubleSupplier speedRPM, double tol, ShooterSubsystem subsystem) {
        m_speed = speedRPM;
        m_tol = tol;
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        
    }

    

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        if (Math.abs(m_speed.getAsDouble()) > 1.0) {
            m_subsystem.setShooterRPMSetpoint(m_speed);
        }
        else {
            m_subsystem.setNetural();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_subsystem.outerShooterRPM() - m_speed.getAsDouble()) < m_tol;
    }
}
