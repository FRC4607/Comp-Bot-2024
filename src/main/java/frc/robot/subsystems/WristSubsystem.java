package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

    /**
     * The subsystem that contains all motors/sensors/encoders in the robot wrist.
     */
    public WristSubsystem() {


    }

    /**
     * Sets the setpoint of the wrist in degrees.
     * 
     * @param newWristSetpoint The new value which updates the old one.
     */
    public void setWristSetpoint(double newWristSetpoint) {

        double wristSetpoint = newWristSetpoint;
    }

    /**
     * sets the max power that can be used by the wrist.
     * 
     * @param newWristPowerCoefficient The new value which updates the old one.
     */
    public void setWristPower(double newWristPowerCoefficient) {

        double wristPowerCoefficient = newWristPowerCoefficient;
    }

    /**
     * Gets the wrist position in degrees.
     */
    public double writstPosition() {
        
        //will need more variables once encoders are added
        return 0.0;
    }
}
