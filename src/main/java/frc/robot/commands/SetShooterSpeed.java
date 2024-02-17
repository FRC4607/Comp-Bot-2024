// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Sets the shooter to a given speed.
 */
public class SetShooterSpeed extends Command {
    private final double m_speed;
    private final ShooterSubsystem m_subsystem;

    /**
     * Creates a new SetShooterSpeed.
     * 
     * @param speedRPM  The speed to set the shooter to in rotations per minute.
     * @param subsystem A reference to the shooter subsystem.
     */
    public SetShooterSpeed(double speedRPM, ShooterSubsystem subsystem) {
        m_speed = speedRPM;
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        m_subsystem.setShooterRPMSetpoint(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_subsystem.outerShooterRPM() - m_speed) < 300.0;
    }
}
