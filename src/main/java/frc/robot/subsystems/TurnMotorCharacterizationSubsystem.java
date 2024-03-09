package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem used in tests involving the the turn motor of a swerve module.
 */
public class TurnMotorCharacterizationSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    final TorqueCurrentFOC m_currentReq = new TorqueCurrentFOC(0).withMaxAbsDutyCycle(0.5).withUpdateFreqHz(1000);
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(1000);

    final StatusSignal<Double> m_encoderPosition;

    /**
     * Creates a new TurnMotorCharacterizationSubsystem.
     * 
     * @param id     The CAN id of the turn motor. Must be a TalonFX.
     * @param canbus The name of the CAN bus the turn and drive motors are on. If
     *               attached to the RoboRIO, specify "rio".
     */
    public TurnMotorCharacterizationSubsystem(int id, String canbus) {
        m_motor = new TalonFX(id, canbus);
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))); // Factory default
                                                                                                     // except for brake
                                                                                                     // mode
        m_motor.getVelocity().setUpdateFrequency(1000);
        m_motor.getClosedLoopError().setUpdateFrequency(1000);
        m_motor.optimizeBusUtilization();
        m_encoder = new CANcoder(id, canbus);
        m_encoder.getConfigurator().apply(new CANcoderConfiguration()); // Factory default
        m_encoderPosition = m_encoder.getPosition();
        m_encoderPosition.setUpdateFrequency(1000);
        m_encoder.optimizeBusUtilization();
    }

    /**
     * Sets the current going to the turn motor.
     * 
     * @param amps The current to apply to the turn motor in Amps.
     */
    public void setCurrent(double amps) {
        m_currentReq.Output = amps;
        m_motor.setControl(m_currentReq);
    }

    /**
     * Sets the turn motor to its neutral state.
     */
    public void setNeutral() {
        m_motor.setControl(m_neutral);
    }

    /**
     * Gets the position of the turn motor.
     * 
     * @return The position of the turn motor in rotations.
     */
    public double getPosition() {
        return m_encoderPosition.waitForUpdate(0.02).getValueAsDouble();
    }
}
