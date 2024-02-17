package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.Calibrations.WristCalibrations;
import frc.robot.Constants.WristConstants;
import frc.robot.util.rev.CANSparkUtil;

/**
 * Subsystem that controls the wrist on the robot.
 */
public class WristSubsystem extends SubsystemBase {
    //private final CANcoder m_encoder;
    private final TalonFX m_motor;

    private final DoubleSupplier m_armAngleSupplier;

    private final MotionMagicTorqueCurrentFOC m_pid;

    private double m_wristPowerCoefficient;

    private double m_setpoint;

    private final StatusSignal<Double> m_position;

    /**
     * Creates a new WristSubsystem.
     */
    public WristSubsystem(DoubleSupplier armAngleSupplier) {
        // CANcoderConfiguration encoder_config = new CANcoderConfiguration();
        // encoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // encoder_config.MagnetSensor.MagnetOffset = Calibrations.WristCalibrations.kEncoderOffset;
        // encoder_config.MagnetSensor.SensorDirection = Constants.WristConstants.kInvertEncoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // m_encoder = new CANcoder(Constants.WristConstants.kCANId);
        // m_encoder.getConfigurator().apply(encoder_config);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.ClosedLoopGeneral.ContinuousWrap = true;
        // config.Feedback.FeedbackRemoteSensorID = Constants.WristConstants.kCANId;
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // config.Feedback.RotorToSensorRatio = Constants.WristConstants.kRotorToSensor;
        // config.Feedback.SensorToMechanismRatio = Constants.WristConstants.kSensorToMechanism;
        config.Feedback.SensorToMechanismRatio = Constants.WristConstants.kRotorToSensor;
        config.MotorOutput.Inverted = Constants.WristConstants.kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kD = Calibrations.WristCalibrations.kD;
        config.Slot0.kP = Calibrations.WristCalibrations.kP;
        config.Slot0.kS = Calibrations.WristCalibrations.kS;
        config.MotionMagic.MotionMagicAcceleration = Calibrations.WristCalibrations.kMotionMagicMaxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = Calibrations.WristCalibrations.kMotionMagicMaxVelocity;
        m_motor = new TalonFX(Constants.WristConstants.kCANId);
        m_motor.getConfigurator().apply(config);

        m_motor.setPosition(0.25);

        m_setpoint = 90;
        m_pid = new MotionMagicTorqueCurrentFOC(0.25);

        //m_position = m_encoder.getPosition();
        m_position = m_motor.getPosition();
        m_position.setUpdateFrequency(500);

        m_wristPowerCoefficient = 1.0;

        m_armAngleSupplier = armAngleSupplier;
    }

    public void periodic() {
        m_pid.Position = (m_setpoint - m_armAngleSupplier.getAsDouble()) / 360.0;
        m_motor.setControl(m_pid);
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
        m_position.refresh();
        return m_position.getValueAsDouble() * 360;
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