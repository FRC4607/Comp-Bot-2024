// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerWheelSubsystem;

public class RunKickerWheel extends Command {
    private final double m_speed;
    private final KickerWheelSubsystem m_subsystem;

    /** Creates a new RunKickerWheel. */
    public RunKickerWheel(double speed, KickerWheelSubsystem subsystem) {
        m_subsystem = subsystem;
        m_speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setKicker(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.setKicker(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
