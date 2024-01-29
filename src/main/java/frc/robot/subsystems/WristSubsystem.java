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

    /**
     * Creates a new WristSubsystem.
     */
    public WristSubsystem() {
        m_motor = new CANSparkFlex(Constants.WristConstants.kCANId, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor((1 / (WristConstants.kRotorToSensor * WristConstants.kSensorToMechanism)) * 360.0);

        CANSparkUtil.ConfigPIDCANSpark(WristCalibrations.kP, WristCalibrations.kI, WristCalibrations.kD,
                WristCalibrations.kFF, m_motor);
        m_pid = m_motor.getPIDController();
        m_pid.setOutputRange(-0.5, 0.5);
    }

    /**
     * Sets the angle of the wrist.
     * 
     * @param theta The angle to target in degrees. Zero is pointing along the Z+
     *              axis and +90deg is pointing along the X+ axis.
     */
    public void setAngle(double theta) {
        m_setpoint = theta;
        m_pid.setReference(theta, ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
    }

    /**
     * Gets the angle of the wrist.
     * 
     * @return The angle of the wrist in degrees. Zero is pointing along the Z+ axis
     *         and +90deg is pointing along the X+ axis.
     */
    public double getAngle() {
        return m_encoder.getPosition();
    }

    public double getPIDSetpoint() {
        return m_setpoint;
    }
}
