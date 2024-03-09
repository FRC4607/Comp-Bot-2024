// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class BumpWrist extends Command {
    private final double m_bump;
    private final WristSubsystem m_subsystem;

    /**
     * Creates a new BumpWrist.
     * 
     * @param bump      The bump to give to the wrist in degrees.
     * @param subsystem A reference to the wrist subsystem.
     */
    public BumpWrist(double bump, WristSubsystem subsystem) {
        m_bump = bump;
        m_subsystem = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double bumpValue = m_subsystem.getPIDSetpoint() + m_bump;
        m_subsystem.setWristSetpoint(() -> {return bumpValue ;});
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
        return true;
    }
}
