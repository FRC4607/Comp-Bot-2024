package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Calibrations.WristCalibrations;
import frc.robot.Constants.WristConstants;
import frc.robot.util.rev.CANSparkUtil;

/**
 * Subsystem that controls the wrist on the robot.
 */
public class WristSubsystem extends SubsystemBase {
    private final CANSparkFlex m_motor;

    private double m_setpoint = 0.0;

    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;

    private double m_wristPowerCoefficient;

    public boolean isRunning = false;

    /**
     * Creates a new WristSubsystem.
     */
    public WristSubsystem() {
        m_motor = new CANSparkFlex(Constants.WristConstants.kCANId, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(
                (1 / (WristConstants.kRotorToSensor * WristConstants.kSensorToMechanism)) * 360.0);

        CANSparkUtil.ConfigPIDCANSpark(WristCalibrations.kP, WristCalibrations.kI, WristCalibrations.kD,
                WristCalibrations.kFF, m_motor);
        m_pid = m_motor.getPIDController();
        m_pid.setOutputRange(-0.5, 0.5);

        m_wristPowerCoefficient = 1.0;
    }

    /**
     * Gets the current setpoint of the wrist's PID controller.
     * 
     * @return The current setpoint of the wrist's PID controller in degrees.
     */
    public double getPIDSetpoint() {
        return m_setpoint;
    }

    /**
     * Sets the setpoint of the wrist in degrees.
     * 
     * @param newWristSetpoint The new setpoint for the wrist (degrees)
     */
    public void setWristSetpoint(double newWristSetpoint) {
        m_setpoint = newWristSetpoint;
        m_pid.setReference(newWristSetpoint, ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
    }

    /**
     * sets the max power that can be used by the wrist.
     * 
     * @param newWristPowerCoefficient The new value which will be multiplied by the
     *                                 amp limits of each motor.
     */
    public void setWristPower(double newWristPowerCoefficient) {
        m_wristPowerCoefficient = newWristPowerCoefficient;
    }

    /**
     * Gets the wrist position in degrees.
     * 
     * @return The wrist's position in degrees.
     */
    public double writstPosition() {
        return m_encoder.getPosition();
    }
}