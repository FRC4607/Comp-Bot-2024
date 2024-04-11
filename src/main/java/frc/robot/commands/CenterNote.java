package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class CenterNote extends Command {
    private final KickerSubsystem m_kicker;
    private final IntakeSubsystem m_intake;

    private boolean m_moveUp = false;
    private int m_count = 0;

    private final int TOTAL_COUNT = 2;
    private final double DISTANCE = 0.75;

    public CenterNote(KickerSubsystem kicker, IntakeSubsystem intake) {
        addRequirements(kicker);
        m_kicker = kicker;
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_moveUp = false;
        m_count = 0;
        m_kicker.setKickerSetpointPosition(m_kicker.getIntakePosition() - DISTANCE);
    }

    @Override
    public void execute() {
        if (m_moveUp) {
            if (!m_intake.hasNote()) {
                m_count++;
                m_moveUp = false;
                if (m_count != TOTAL_COUNT) {
                    m_kicker.setKickerSetpointPosition(m_kicker.getIntakePosition() - DISTANCE);
                }
            }
        } else {
            if (m_intake.hasNote()) {
                m_moveUp = true;
                m_kicker.setKickerSetpointPosition(m_kicker.getIntakePosition() + DISTANCE);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_count >= TOTAL_COUNT;
    }
}
