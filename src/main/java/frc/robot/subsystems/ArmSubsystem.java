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
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANcoder m_encoder;
    private final TalonFX m_front;
    private final TalonFX m_rear;
    
    private final Follower m_follow;
    private final MotionMagicTorqueCurrentFOC m_motionMagic;
    private final NeutralOut m_neutral;

    private double m_armPowerCoefficient;

    private final StatusSignal<Double> m_armPos;
    private final DoubleLogEntry m_armPosLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/pos");
    private final StatusSignal<Double> m_armSetpoint;
    private final DoubleLogEntry m_armSetpointLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/setpoint");
    private final StatusSignal<Double> m_armTorquePerMotor;
    private final DoubleLogEntry m_armTorquePerMotorLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/torque_per_motor");
    private final StatusSignal<Double> m_armVelocity;
    private final DoubleLogEntry m_armVelocityLog = new DoubleLogEntry(DataLogManager.getLog(), "arm/vel");

    private final StatusSignal<Double> m_deviceTempSecondaryFront;
    private final DoubleLogEntry m_deviceTempSecondaryLogFront = new DoubleLogEntry(DataLogManager.getLog(), "arm/front/temp_secondary");
    private final StatusSignal<Double> m_deviceTempFront;
    private final DoubleLogEntry m_deviceTempLogFront = new DoubleLogEntry(DataLogManager.getLog(), "arm/front/temp");
    private final StatusSignal<Double> m_processorTempFront;
    private final DoubleLogEntry m_processorTempLogFront = new DoubleLogEntry(DataLogManager.getLog(), "arm/front/temp");

    private final StatusSignal<Boolean> m_bootDuringEnableFront;
    private final IntegerLogEntry m_bootDuringEnableLogFront = new IntegerLogEntry(DataLogManager.getLog(), "arm/front/fault_boot_during_enable");
    /**
     * The subsystem which contains all the motors/encoders/sensors on the arm of
     * the robot.
     */
    public ArmSubsystem() {
        CANcoderConfiguration encoder_config = new CANcoderConfiguration();
        encoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder_config.MagnetSensor.MagnetOffset = ArmCalibrations.kEncoderOffset;
        encoder_config.MagnetSensor.SensorDirection = ArmConstants.kInvertEncoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        m_encoder = new CANcoder(ArmConstants.kFrontCANId);
        m_encoder.getConfigurator().apply(encoder_config);
        m_encoder.getPosition().setUpdateFrequency(500.0);
        m_encoder.getVelocity().setUpdateFrequency(500.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = ArmConstants.kFrontCANId;
        config.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensor;
        config.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanism;
        config.MotorOutput.Inverted = ArmConstants.kInvertMotors ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // config.Slot0.kS = ArmCalibrations.kS; These values need to be retuned
        // config.Slot0.kG = ArmCalibrations.kG;
        // config.Slot0.kP = ArmCalibrations.kP;
        // config.Slot0.kD = ArmCalibrations.kD;
        config.Slot0.kG = 25.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kReverseSoftLimit;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmCalibrations.kMaxSpeedMotionMagic;
        config.MotionMagic.MotionMagicAcceleration = ArmCalibrations.kMaxAccelerationMotionMagic;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ArmCalibrations.kMaxArmCurrentPerMotor;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmCalibrations.kMaxArmCurrentPerMotor;

        m_front = new TalonFX(ArmConstants.kFrontCANId);
        m_front.getConfigurator().apply(config);
        m_rear = new TalonFX(ArmConstants.kRearCANId);
        m_rear.getConfigurator().apply(config);

        m_follow = new Follower(ArmConstants.kFrontCANId, false);
        m_rear.setControl(m_follow);

        m_motionMagic = new MotionMagicTorqueCurrentFOC(0);

        m_armPos = m_front.getPosition();
        m_armPos.setUpdateFrequency(50.0);

        m_neutral = new NeutralOut();

        m_deviceTempSecondaryFront = m_front.getAncillaryDeviceTemp();
        m_deviceTempSecondaryFront.setUpdateFrequency(4.0);
        m_armSetpoint = m_front.getClosedLoopReference();
        m_armSetpoint.setUpdateFrequency(50.0);
        m_deviceTempFront = m_front.getDeviceTemp();
        m_deviceTempFront.setUpdateFrequency(4.0);
        m_processorTempFront = m_front.getProcessorTemp();
        m_processorTempFront.setUpdateFrequency(4.0);
        m_armTorquePerMotor = m_front.getTorqueCurrent();
        m_armTorquePerMotor.setUpdateFrequency(50.0);
        m_armVelocity = m_front.getVelocity();
        m_armVelocity.setUpdateFrequency(50.0);

        m_bootDuringEnableFront = m_front.getStickyFault_BootDuringEnable();
        m_bootDuringEnableFront.setUpdateFrequency(4.0);
    }

    /**
     * Sets the setpoint (degrees) for the arm to use with PID control.
     * 
     * @param newArmSetpoint The new setpoint (degrees) which updates the old one.
     0deg points the arm out along the +X axis. +90deg points the arm up along the +Z axis.
     */
    public void setArmSetpoint(double newArmSetpoint) {
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
     * @return the arm position in degrees.
     */
    public double armPosition() {
        m_armPos.refresh();
        return m_armPos.getValueAsDouble() * 360.0;
    }
}
