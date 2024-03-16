package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    // private final CANcoder m_leftEncoder;
    // private final CANcoder m_rightEncoder

    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    /**
     * The subsystem which contains all motors/sensors/encoers on the robot
     * climbers.
     */
    public ClimberSubsystem() {

        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.kSupplyAmpLimit;
        
        
        m_leftMotor = new TalonFX(ClimberConstants.kLeftCANId, "kachow");
        m_leftMotor.getConfigurator().apply(config);
        m_rightMotor = new TalonFX(ClimberConstants.kRightCANId, "kachow");
        m_rightMotor.getConfigurator().apply(config);

        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);
    }

    /**
     * Sets the setpoint of the left climber to be used by open loop control.
     * 
     * @param speed The new speed which will update the old one.
     */
    public void setLeftClimberSpeed(double speed) {

        m_leftMotor.set(speed);
    }

    /**
     * Sets the setpoint (Inches) of the right climber to be used by open loop control.
     * 
     * @param speed The new setpoint (inches) which will upate the
     *                                old one.
     */
    public void setRightClimberSpeed(double speed) {

        m_rightMotor.set(speed);
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
        return 0.0;
    }

    /**
     * gets the position of the right climber in inches
     * 
     * @return right climber length in inches
     */
    public double rightClimberPosition() {

        // will need more variables once encoders are added
        return 0.0;
    }
}
