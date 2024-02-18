package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

/** The subsystem that controls the robot's arm. */
public class ArmSubsystem extends SubsystemBase {
    private final CANcoder m_encoder;
    private final TalonFX m_front;
    private final TalonFX m_rear;

    private final Follower m_follow;
    private final MotionMagicTorqueCurrentFOC m_motionMagic;
    private final NeutralOut m_neutral;

    private double m_armPowerCoefficient;

    private final TalonFXStandardSignalLogger m_frontLog;
    private final TalonFXStandardSignalLogger m_rearLog;

    private final StatusSignal<Double> m_armSetpoint;
    private final DoubleLogEntry m_armSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/setpoint");

    private final DoubleLogEntry m_armGoalLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/goal");

    /**
     * Creates a new ArmSubsystem object.
     */
    public ArmSubsystem() {
        // Set up CANcoder
        CANcoderConfiguration encoder_config = new CANcoderConfiguration();
        encoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder_config.MagnetSensor.MagnetOffset = ArmCalibrations.kEncoderOffset;
        encoder_config.MagnetSensor.SensorDirection = ArmConstants.kInvertEncoder
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        m_encoder = new CANcoder(ArmConstants.kFrontCANID);
        m_encoder.getConfigurator().apply(encoder_config);
        m_encoder.getPosition().setUpdateFrequency(500.0);
        m_encoder.getVelocity().setUpdateFrequency(500.0);

        // Set up arm motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = ArmConstants.kFrontCANID;
        config.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensor;
        config.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanism;
        config.MotorOutput.Inverted = ArmConstants.kInvertMotors ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kS = ArmCalibrations.kS;
        config.Slot0.kG = ArmCalibrations.kG;
        config.Slot0.kP = ArmCalibrations.kP;
        config.Slot0.kD = ArmCalibrations.kD;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kReverseSoftLimit;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmCalibrations.kMaxSpeedMotionMagic;
        config.MotionMagic.MotionMagicAcceleration = ArmCalibrations.kMaxAccelerationMotionMagic;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ArmCalibrations.kMaxArmCurrentPerMotor;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmCalibrations.kMaxArmCurrentPerMotor;

        m_front = new TalonFX(ArmConstants.kFrontCANID);
        m_front.getConfigurator().apply(config);
        m_rear = new TalonFX(ArmConstants.kRearCANID);
        m_rear.getConfigurator().apply(config);

        m_follow = new Follower(ArmConstants.kFrontCANID, false);
        m_rear.setControl(m_follow);

        // Create control requests
        m_motionMagic = new MotionMagicTorqueCurrentFOC(0);
        m_neutral = new NeutralOut();

        // Set up logging
        m_frontLog = new TalonFXStandardSignalLogger(m_front, "/arm/front");
        m_rearLog = new TalonFXStandardSignalLogger(m_front, "/arm/rear");

        // Set up logging of the arm's setpoint generated by Motion Magic.
        m_armSetpoint = m_front.getClosedLoopReference();
        m_armSetpoint.setUpdateFrequency(50.0);
        Robot.addSignals(m_armSetpoint);
    }

    public void periodic() {
        m_armSetpointLog.append(m_armSetpoint.getValueAsDouble(),
                (long) (m_armSetpoint.getTimestamp().getTime() * 1e6));
        m_frontLog.log();
        m_rearLog.log();
    }

    /**
     * Sets the setpoint (degrees) for the arm to use with PID control.
     * 
     * @param newArmSetpoint The new setpoint (degrees) which updates the old one.
     *                       0deg points the arm out along the +X axis. +90deg
     *                       points the arm up along the +Z axis.
     */
    public void setArmSetpoint(double newArmSetpoint) {
        m_armGoalLog.append(newArmSetpoint);
        m_front.setControl(m_motionMagic.withPosition(newArmSetpoint / 360.0));
    }

    /**
     * Sets the maximum amount of power that can be used by the arm.
     * 
     * @param newArmPowerCoefficient The new value which will be multiplied by the
     *                               amp limits of each motor.
     */
    public void setArmPower(double newArmPowerCoefficient) {
        m_armPowerCoefficient = newArmPowerCoefficient;
    }

    /** Sets the output of the arm to neutral. */
    public void setNeutral() {
        m_front.setControl(m_neutral);
    }

    /**
     * Returns the arm position in degrees.
     * 
     * @return The arm position in degrees.
     */
    public double armPosition() {
        return m_frontLog.m_pos.getValueAsDouble() * 360.0;
    }
}
