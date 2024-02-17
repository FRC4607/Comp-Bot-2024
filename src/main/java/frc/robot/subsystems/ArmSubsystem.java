package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
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

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
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
    private final DoubleLogEntry m_deviceTempLogFront = new DoubleLogEntry(DataLogManager.getLog(), "arm/front/temp_device");
    private final StatusSignal<Double> m_processorTempFront;
    private final DoubleLogEntry m_processorTempLogFront = new DoubleLogEntry(DataLogManager.getLog(), "arm/front/temp_proc");

    private final StatusSignal<Boolean> m_bootDuringEnableFront;
    private final BooleanLogEntry m_bootDuringEnableLogFront = new BooleanLogEntry(DataLogManager.getLog(), "arm/front/fault_boot_during_enable");
    private final StatusSignal<Boolean> m_hardwareFront;
    private final BooleanLogEntry m_hardwareFrontLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/front/fault_hardware");
    private final StatusSignal<Boolean> m_deviceTempFaultFront;
    private final BooleanLogEntry m_deviceTempFaultFrontLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/front/fault_motor_temp");
    private final StatusSignal<Boolean> m_procTempFaultFront;
    private final BooleanLogEntry m_procTempFaultFrontLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/front/fault_proc_temp");

    private final StatusSignal<Double> m_deviceTempSecondaryBack;
    private final DoubleLogEntry m_deviceTempSecondaryLogBack = new DoubleLogEntry(DataLogManager.getLog(), "arm/back/temp_secondary");
    private final StatusSignal<Double> m_deviceTempBack;
    private final DoubleLogEntry m_deviceTempLogBack = new DoubleLogEntry(DataLogManager.getLog(), "arm/back/temp_device");
    private final StatusSignal<Double> m_processorTempBack;
    private final DoubleLogEntry m_processorTempLogBack = new DoubleLogEntry(DataLogManager.getLog(), "arm/back/temp_proc");

    private final StatusSignal<Boolean> m_bootDuringEnableBack;
    private final BooleanLogEntry m_bootDuringEnableLogBack = new BooleanLogEntry(DataLogManager.getLog(), "arm/back/fault_boot_during_enable");
    private final StatusSignal<Boolean> m_hardwareBack;
    private final BooleanLogEntry m_hardwareBackLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/back/fault_hardware");
    private final StatusSignal<Boolean> m_deviceTempFaultBack;
    private final BooleanLogEntry m_deviceTempFaultBackLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/back/fault_motor_temp");
    private final StatusSignal<Boolean> m_procTempFaultBack;
    private final BooleanLogEntry m_procTempFaultBackLog = new BooleanLogEntry(DataLogManager.getLog(), "arm/back/fault_proc_temp");
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
        m_hardwareFront = m_front.getStickyFault_Hardware();
        m_hardwareFront.setUpdateFrequency(4.0);
        m_deviceTempFaultFront = m_front.getStickyFault_DeviceTemp();
        m_deviceTempFaultFront.setUpdateFrequency(4.0);
        m_procTempFaultFront = m_front.getStickyFault_ProcTemp();
        m_procTempFaultFront.setUpdateFrequency(4.0);

        m_deviceTempSecondaryBack = m_rear.getAncillaryDeviceTemp();
        m_deviceTempSecondaryBack.setUpdateFrequency(4.0);
        m_deviceTempBack = m_rear.getDeviceTemp();
        m_deviceTempBack.setUpdateFrequency(4.0);
        m_processorTempBack = m_rear.getProcessorTemp();
        m_processorTempBack.setUpdateFrequency(4.0);

        m_bootDuringEnableBack = m_rear.getStickyFault_BootDuringEnable();
        m_bootDuringEnableBack.setUpdateFrequency(4.0);
        m_hardwareBack = m_rear.getStickyFault_Hardware();
        m_hardwareBack.setUpdateFrequency(4.0);
        m_deviceTempFaultBack = m_rear.getStickyFault_DeviceTemp();
        m_deviceTempFaultBack.setUpdateFrequency(4.0);
        m_procTempFaultBack = m_rear.getStickyFault_ProcTemp();
        m_procTempFaultBack.setUpdateFrequency(4.0);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            m_armPos,
            m_armSetpoint,
            m_armTorquePerMotor,
            m_armVelocity,
            m_bootDuringEnableBack,
            m_bootDuringEnableFront,
            m_deviceTempBack,
            m_deviceTempFaultBack,
            m_deviceTempFaultFront,
            m_deviceTempFront,
            m_deviceTempSecondaryBack,
            m_deviceTempSecondaryFront,
            m_hardwareBack,
            m_hardwareFront,
            m_procTempFaultBack,
            m_procTempFaultFront,
            m_processorTempBack,
            m_processorTempFront
        );
        m_armPosLog.append(m_armPos.getValueAsDouble(), (long) (m_armPos.getTimestamp().getTime() * 1e6));
        m_armSetpointLog.append(m_armSetpoint.getValueAsDouble(), (long) (m_armSetpoint.getTimestamp().getTime() * 1e6));
        m_armTorquePerMotorLog.append(m_armTorquePerMotor.getValueAsDouble(), (long) (m_armTorquePerMotor.getTimestamp().getTime() * 1e6));
        m_armVelocityLog.append(m_armVelocity.getValueAsDouble(), (long) (m_armVelocity.getTimestamp().getTime() * 1e6));
        m_deviceTempLogBack.append(m_deviceTempBack.getValueAsDouble(), (long) (m_deviceTempBack.getTimestamp().getTime() * 1e6));
        m_deviceTempLogFront.append(m_deviceTempFront.getValueAsDouble(), (long) (m_deviceTempFront.getTimestamp().getTime() * 1e6));
        m_deviceTempSecondaryLogBack.append(m_deviceTempSecondaryBack.getValueAsDouble(), (long) (m_deviceTempSecondaryBack.getTimestamp().getTime() * 1e6));
        m_deviceTempSecondaryLogFront.append(m_deviceTempSecondaryFront.getValueAsDouble(), (long) (m_deviceTempSecondaryFront.getTimestamp().getTime() * 1e6));
        m_processorTempLogBack.append(m_processorTempBack.getValueAsDouble(), (long) (m_processorTempBack.getTimestamp().getTime() * 1e6));
        m_processorTempLogFront.append(m_processorTempFront.getValueAsDouble(), (long) (m_processorTempFront.getTimestamp().getTime() * 1e6));

        m_bootDuringEnableLogBack.append(m_bootDuringEnableBack.getValue(), (long) (m_bootDuringEnableBack.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnableBack.getValue()) {
            m_rear.clearStickyFault_BootDuringEnable();
        }
        m_bootDuringEnableLogFront.append(m_bootDuringEnableFront.getValue(), (long) (m_bootDuringEnableFront.getTimestamp().getTime() * 1e6));
        if (m_bootDuringEnableBack.getValue()) {
            m_rear.clearStickyFault_BootDuringEnable();
        }
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
        return m_armPos.getValueAsDouble() * 360.0;
    }
}
