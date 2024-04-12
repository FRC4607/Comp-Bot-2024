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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/**
 * Subsystem that controls the wrist on the robot.
 */
public class WristSubsystem extends SubsystemBase {
    private final CANcoder m_encoder;
    private final TalonFX m_motor;

    private final DoubleSupplier m_armAngleSupplier;

    private final MotionMagicTorqueCurrentFOC m_pid;

    private double m_wristPowerCoefficient;

    private DoubleSupplier m_setpoint;

    private boolean m_ignoreArm = false;

    private final TalonFXStandardSignalLogger m_log;

    private final StatusSignal<Double> m_armSetpoint;
    private final DoubleLogEntry m_wristSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "/wrist/setpoint");

    private final DoubleLogEntry m_wristGoalLog = new DoubleLogEntry(DataLogManager.getLog(), "/wrist/goal");

    /**
     * Creates a new WristSubsystem.
     */
    public WristSubsystem(DoubleSupplier armAngleSupplier) {
        CANcoderConfiguration encoder_config = new CANcoderConfiguration();
        encoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder_config.MagnetSensor.MagnetOffset = Calibrations.WristCalibrations.kEncoderOffset;
        encoder_config.MagnetSensor.SensorDirection = Constants.WristConstants.kInvertEncoder
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        m_encoder = new CANcoder(Constants.WristConstants.kCANID, "kachow");
        m_encoder.getConfigurator().apply(encoder_config);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.Feedback.FeedbackRemoteSensorID = Constants.WristConstants.kCANID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        config.Feedback.RotorToSensorRatio = Constants.WristConstants.kRotorToSensor;
        config.Feedback.SensorToMechanismRatio = Constants.WristConstants.kSensorToMechanism;
        config.MotorOutput.Inverted = Constants.WristConstants.kInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kD = Calibrations.WristCalibrations.kD;
        config.Slot0.kP = Calibrations.WristCalibrations.kP;
        config.Slot0.kS = Calibrations.WristCalibrations.kS;
        config.Slot0.kG = Calibrations.WristCalibrations.kG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot1.kD = Calibrations.WristCalibrations.kD;
        config.Slot1.kP = Calibrations.WristCalibrations.kP;
        config.Slot1.kS = Calibrations.WristCalibrations.kS;

        config.MotionMagic.MotionMagicAcceleration = Calibrations.WristCalibrations.kMotionMagicMaxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = Calibrations.WristCalibrations.kMotionMagicMaxVelocity;
        m_motor = new TalonFX(Constants.WristConstants.kCANID, "kachow");
        m_motor.getConfigurator().apply(config);

        m_setpoint = () -> 100.0;
        m_pid = new MotionMagicTorqueCurrentFOC(100.0 / 360.0);

        // m_position = m_encoder.getPosition();
        m_log = new TalonFXStandardSignalLogger(m_motor, "/wrist");

        m_armSetpoint = m_motor.getClosedLoopReference();
        m_armSetpoint.setUpdateFrequency(50.0);
        Robot.addSignalsCaniv(m_armSetpoint);

        m_wristPowerCoefficient = 1.0;

        m_armAngleSupplier = armAngleSupplier;
    }

    public void periodic() {
        if (m_ignoreArm) {
            m_pid.Position = m_setpoint.getAsDouble() / 360.0;
        } else {
            m_pid.Position = (m_setpoint.getAsDouble() - m_armAngleSupplier.getAsDouble()) / 360.0;
        }
        // m_pid.FeedForward = Math.cos(Math.toRadians(getWristPosition())) *
        // Calibrations.WristCalibrations.kG;
        m_pid.Slot = m_armAngleSupplier.getAsDouble() > 2.0 ? 1 : 0;
        m_motor.setControl(m_pid);

        m_wristGoalLog.append(m_pid.Position);
        m_wristSetpointLog.append(m_armSetpoint.getValueAsDouble());

        //SmartDashboard.putBoolean("Using Arm", m_ignoreArm);
        SmartDashboard.putNumber("Wrist Angle", getWristPosition());
        //SmartDashboard.putNumber("Wrist Target", m_setpoint.getAsDouble());
        SmartDashboard.putNumber("Arm Angle", m_armAngleSupplier.getAsDouble());
        m_log.log();
    }

    /**
     * Gets the current setpoint of the wrist's PID controller.
     * 
     * @return The current setpoint of the wrist's PID controller in degrees.
     */
    public double getPIDSetpoint() {
        return m_setpoint.getAsDouble();
    }

    /**
     * Sets the setpoint of the wrist in degrees.
     * 
     * @param newWristSetpoint The new setpoint for the wrist in degrees. 0deg
     *                         points along +X, 90deg points along +Z.
     */
    public void setWristSetpoint(DoubleSupplier newWristSetpoint) {
        m_setpoint = newWristSetpoint;
    }

    /**
     * Sets the max power that can be used by the wrist.
     * 
     * @param newWristPowerCoefficient The new value which will be multiplied by the
     *                                 amp limits of each motor.
     */
    public void setWristPower(double newWristPowerCoefficient) {
        m_wristPowerCoefficient = newWristPowerCoefficient;
    }

    /**
     * Return's the wrist position without compensation for the angle of the arm.
     * 
     * @return The wrist's uncompensated position in CCW+ degrees.
     */
    public double getRawWristPosition() {
        return m_log.m_pos.getValueAsDouble() * 360;
    }

    /**
     * Gets the wrist position in degrees.
     * 
     * @return The wrist's position in degrees, where 0deg points along +X and 90deg
     *         points along +Z.
     */
    public double getWristPosition() {
        return getRawWristPosition() + m_armAngleSupplier.getAsDouble();
    }

    public void setIgnoreArm(boolean ignoreArm) {
        m_ignoreArm = ignoreArm;
    }
}