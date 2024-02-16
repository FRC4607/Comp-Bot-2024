// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Controls the intake and kicker so they are at the same linear speed.
 */
public class RunIntakeSyncAuto extends Command {
    private final IntakeSubsystem m_intake;
    private final KickerSubsystem m_kicker;
    private final DoubleSupplier m_power;

    private boolean m_hadNote;

    private static final double MAX_SURFACE_SPEED = 3000.0;

    /**
     * Creates a new RunIntakeSyncAuto. Provides the same functionality as {@link RunIntakesSync}, but exits immidiately without setting speeds to 0 on exit.
     * 
     * @param speed The percentage of the max surface speed to run the kicker and intake at in the range [-1, 1].
     */
    public RunIntakeSyncAuto(DoubleSupplier power, IntakeSubsystem intake, KickerSubsystem kicker) {
        m_power = power;
        m_intake = intake;
        m_kicker = kicker;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake, m_kicker);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_hadNote = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        double surfaceSpeed = m_power.getAsDouble() * MAX_SURFACE_SPEED;
        m_intake.setIntakeSetpoint(surfaceSpeed);
        m_kicker.setKickerSetpoint(surfaceSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeSetpoint(0);
        m_kicker.setKickerSetpoint(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (m_intake.hasNote()) {
            m_hadNote = true;    
        } else if (m_hadNote) {
            if (m_intake.hasNote() == false) {
                return true;
            }
        }

        return false;    
    }
}
