package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Subsystem used in coupling characterization tests that includes two motors.
 */
public class CouplingCharacterizationSubsystem extends TurnMotorCharacterizationSubsystem {
    private final TalonFX m_drive;
    private final StatusSignal<Double> m_drivePos;

    public CouplingCharacterizationSubsystem(int turnid, int driveid, String canbus) {
        super(turnid, canbus);
        m_currentReq.MaxAbsDutyCycle = 0.05;

        m_drive = new TalonFX(driveid, canbus);
        m_drive.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
        m_drivePos = m_drive.getPosition();
        m_drivePos.setUpdateFrequency(1000);
        m_drive.optimizeBusUtilization();
    }

    /**
     * Updates the signals in this object.
     */
    public void updateSignals() {
        BaseStatusSignal.waitForAll(0.002, m_drivePos, m_encoderPosition);
    }

    /**
     * Gets the turn encoder's position
     * @return The turn encoder's position in rotations, CCW positive from the top of the module down.
     */
    public double getTurnPosition() {
        return m_encoderPosition.getValueAsDouble();
    }

    /**
     * Gets the drive encoder's position.
     * @return The drive encoder's position in rotations at the motor, CCW positive when looking at the side of the motor with the rotor.
     */
    public double getDrivePosition() {
        return m_drivePos.getValueAsDouble();
    }
}
