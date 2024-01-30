package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public boolean m_isRunning = false;

    /**
     * The subsystem which contains all the motors/encoders/sensors on the arm of
     * the robot.
     */
    public ArmSubsystem() {

    }

    /**
     * Sets the setpoint (degrees) for the arm to use with PID control.
     * 
     * @param newArmSetpoint The new setpoint (degrees) which updates the old one.
     */
    public static void setArmSetpoint(double newArmSetpoint) {
        double armSetpoint = newArmSetpoint;
    }

    /**
     * Sets the maximum amount of power that can be used by the arm.
     * 
     * @param newArmPowerCoefficient The new value which will be multiplied by the
     *                               amp limits of each motor.
     */
    public void setArmPower(double newArmPowerCoefficient) {
        double armPowerCoefficient = newArmPowerCoefficient;
    }

    /**
     * Returns the arm position in degrees.
     * 
     * @return the arm position in degrees.
     */
    public double armPosition() {

        // more variables will be needed once encoders are programmed
        return 0.0;
    }
}
