package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX m_front;
    private final TalonFX m_rear;
    
    private final Follower m_follow;
    private final MotionMagicTorqueCurrentFOC m_motionMagic;
    private final NeutralOut m_neutral;

    private double m_armPowerCoefficient;

    private final StatusSignal<Double> m_armPos;
    /**
     * The subsystem which contains all the motors/encoders/sensors on the arm of
     * the robot.
     */
    public ArmSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensor;
        config.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanism;
        config.MotorOutput.Inverted = ArmConstants.kInvertMotors ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = ArmCalibrations.kG;
        config.Slot0.kS = ArmCalibrations.kS;
        config.Slot0.kP = ArmCalibrations.kP;
        config.Slot0.kD = ArmCalibrations.kD;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kReverseSoftLimit;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmCalibrations.kMaxSpeedMotionMagic;
        config.MotionMagic.MotionMagicAcceleration = ArmCalibrations.kMaxAccelerationMotionMagic;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        m_front = new TalonFX(ArmConstants.kFrontCANId);
        m_front.getConfigurator().apply(config);
        m_rear = new TalonFX(ArmConstants.kRearCANId);
        m_rear.getConfigurator().apply(config);

        m_follow = new Follower(ArmConstants.kFrontCANId, false);
        m_rear.setControl(m_follow);

        m_front.setPosition(0.0);
        m_motionMagic = new MotionMagicTorqueCurrentFOC(0);

        m_armPos = m_front.getPosition();

        m_neutral = new NeutralOut();
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
