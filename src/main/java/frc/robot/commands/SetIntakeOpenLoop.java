// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Sets the intake using open loop control.
 */
public class SetIntakeOpenLoop extends Command {
    private final IntakeSubsystem m_intake;
    private final DoubleSupplier m_power;

    /**
     * Creates a new SetIntakeOpenLoop.
     * 
     * @param speed The open loop speed to run the kicker at in the range [-1, 1].
     */
    public SetIntakeOpenLoop(DoubleSupplier power, IntakeSubsystem intake) {
        m_power = power;
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        m_intake.setPower(m_power.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
