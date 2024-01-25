package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.rev.CANSparkUtil;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkFlex m_motor;

    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;
    public WristSubsystem() {
        m_motor = new CANSparkFlex(Constants.WristConstants.kCANId, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(Constants.WristConstants.kRotorToSensor * Constants.WristConstants.kSensorToMechanism);

        CANSparkUtil.ConfigPIDCANSpark(3, 0, 0, 0, m_motor);
        m_pid = m_motor.getPIDController();
        m_pid.setOutputRange(-0.1, 0.1);
    }

    /** Sets the angle of the wrist. Zero is pointing along the Z+ axis and +90deg is pointing along the X+ axis. **/
    public void setAngle(Measure<Angle> theta) {
        m_pid.setReference(theta.in(Units.Rotations), ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
    }

    /** Gets the angle of the wrist. Zero is pointing along the Z+ axis and +90deg is pointing along the X+ axis. **/
    public Measure<Angle> getAngle() {
        return Units.Rotations.of(m_encoder.getPosition());
    }
}
