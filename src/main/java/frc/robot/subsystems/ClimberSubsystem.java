package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;

public class ClimberSubsystem extends SubsystemBase {

    // private final CANcoder m_leftEncoder;
    // private final CANcoder m_rightEncoder

    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    private final TalonFXStandardSignalLogger m_leftLogger;
    private final TalonFXStandardSignalLogger m_rightLogger;

    private final MotionMagicTorqueCurrentFOC m_control;

    /**
     * The subsystem which contains all motors/sensors/encoers on the robot
     * climbers.
     */
    public ClimberSubsystem() {

        m_control = new MotionMagicTorqueCurrentFOC(0);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.kSupplyAmpLimit;
        config.MotionMagic.MotionMagicAcceleration = 647.25;
        config.MotionMagic.MotionMagicCruiseVelocity = 98;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 32;
        config.Slot0.kD = 6.4;
        config.Slot0.kS = 1.75;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 85.0;
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotor = new TalonFX(ClimberConstants.kLeftCANId, "kachow");
        m_leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_rightMotor = new TalonFX(ClimberConstants.kRightCANId, "kachow");
        m_rightMotor.getConfigurator().apply(config);

        m_leftLogger = new TalonFXStandardSignalLogger(m_leftMotor, "/climber/left");
        m_rightLogger = new TalonFXStandardSignalLogger(m_rightMotor, "/climber/right");
    }

    @Override
    public void periodic() {
        m_control.FeedForward = ((leftClimberPosition() + rightClimberPosition())/2.0) < 20.0 ? 11.0 : 0.0;

        m_leftMotor.setControl(m_control);
        m_rightMotor.setControl(m_control);

        m_leftLogger.log();
        m_rightLogger.log();
    }



    /**
     * Sets the setpoint (Inches) of the right climber to be used by open loop control.
     * 
     * @param speed The new setpoint (inches) which will upate the
     *                                old one.
     */
    public void setClimberPosition(double position) {

        m_control.Position = position;
    }

    /**
     * Sets the max power available to be used by the climbers.
     * 
     * @param newClimberPowerCoefficient The new value which will be multiplied by
     *                                   the amp limits of each motor.
     */
    public void setClimberPower(double newClimberPowerCoefficient) {

        double climberPowerCoefficient = newClimberPowerCoefficient;
    }

    /**
     * gets the position of the left climber in inches
     * 
     * @return left climber length in inches
     */
    public double leftClimberPosition() {

        // will need more variables once encoders are added
        return m_leftLogger.m_pos.getValueAsDouble();
    }

    /**
     * gets the position of the right climber in inches
     * 
     * @return right climber length in inches
     */
    public double rightClimberPosition() {

        // will need more variables once encoders are added
        return m_rightLogger.m_pos.getValueAsDouble();
    }
}
