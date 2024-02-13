// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Command that sets the kicker wheel to a given speed.
 */
public class RunKickerWheel extends Command {
    private final double m_speed;
    private final KickerSubsystem m_subsystem;

    /**
     * Creates a new RunKickerWheel.
     * 
     * @param speed The open loop speed to run the kicker at in the range [-1, 1].
     */
    public RunKickerWheel(double speed, KickerSubsystem subsystem) {
        m_subsystem = subsystem;
        m_speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setOpenLoop(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.setOpenLoop(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
