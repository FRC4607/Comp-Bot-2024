package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double m_setpoint = 90.0;

    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;

    private final DoubleSupplier m_armAngleSupplier;

    private double m_wristPowerCoefficient;

    /**
     * Creates a new WristSubsystem.
     */
    public WristSubsystem(DoubleSupplier armAngleSupplier) {
        m_motor = new CANSparkFlex(Constants.WristConstants.kCANId, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(IdleMode.kBrake);

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(
                (1 / (WristConstants.kRotorToSensor * WristConstants.kSensorToMechanism)) * 360.0);
        m_encoder.setPosition(90.0);

        CANSparkUtil.ConfigPIDCANSpark(WristCalibrations.kP, WristCalibrations.kI, WristCalibrations.kD,
                WristCalibrations.kFF, m_motor);
        m_pid = m_motor.getPIDController();
        m_pid.setOutputRange(-0.5, 0.5);

        m_wristPowerCoefficient = 1.0;

        m_armAngleSupplier = armAngleSupplier;
    }

    public void periodic() {
        m_pid.setReference(m_setpoint - m_armAngleSupplier.getAsDouble(), ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
        SmartDashboard.putNumber("Wrist Angle", m_setpoint);
        SmartDashboard.putNumber("Arm Angle", m_armAngleSupplier.getAsDouble());
        SmartDashboard.putNumber("Setpoint", m_setpoint - m_armAngleSupplier.getAsDouble());
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

    public double getRawWristPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Gets the wrist position in degrees.
     * 
     * @return The wrist's position in degrees.
     */
    public double getWristPosition() {
        return getRawWristPosition() + m_armAngleSupplier.getAsDouble();
    }
}