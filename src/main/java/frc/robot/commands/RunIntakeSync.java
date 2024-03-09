// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Controls the intake and kicker so they run at the same linear speed.
 */
public class RunIntakeSync extends Command {
    private final IntakeSubsystem m_intake;
    private final KickerSubsystem m_kicker;
    private final DoubleSupplier m_power;
    private final LEDSubsystem m_leds;
    private boolean m_hadNote;
    private int i;

    private static final double MAX_SURFACE_SPEED = 3000.0;

    private final boolean m_ignoreBeam;

    /**
     * Creates a new RunIntakeSync.
     * 
     * @param speed      The percentage of the max surface speed to run the kicker
     *                   and intake at in the range [-1, 1].
     * @param intake     A refernce to the
     *                   {@link frc.robot.subsystems.IntakeSubsystem} object.
     * @param kicker     A refernce to the
     *                   {@link frc.robot.subsystems.KickerSubsystem} object.
     * @param ignoreBeam Whether or not to ignore the beam break sensor and end
     *                   immidiately. Useful for stopping the intake in an
     *                   autonomous routine.
     */
    public RunIntakeSync(DoubleSupplier power, IntakeSubsystem intake, KickerSubsystem kicker, LEDSubsystem leds, boolean ignoreBeam) {
        m_power = power;
        m_intake = intake;
        m_kicker = kicker;
        m_ignoreBeam = ignoreBeam;
        m_leds = leds;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake, m_kicker);
    }

    /**
     * Creates a new RunIntakeSync.
     * 
     * @param speed  The percentage of the max surface speed to run the kicker and
     *               intake at in the range [-1, 1].
     * @param intake A refernce to the {@link frc.robot.subsystems.IntakeSubsystem}
     *               object.
     * @param kicker A refernce to the {@link frc.robot.subsystems.KickerSubsystem}
     *               object.
     */
    public RunIntakeSync(DoubleSupplier power, IntakeSubsystem intake, KickerSubsystem kicker, LEDSubsystem leds) {
        this(power, intake, kicker, leds, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_hadNote = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
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
        if (m_ignoreBeam) {
            return true;
        } else {
            // If we have a note, reset the counter.
            if (m_intake.hasNote()) {
                m_hadNote = true;
                i = 0;
            } else if (m_hadNote) {
                // Wait 1 cycles before exiting for debouncing and delay.
                if (!m_intake.hasNote()) {
                    i++;
                }

                if (i >= 1) {
                    m_leds.setLED(0, 255, 0, 0);
                    return true;
                }
            }
            return false;
        }
    }
}
