package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    /**
     * The subsystem which contains all the motors/encoders/sensors on the arm of the robot.
     */
    public ArmSubsystem () {


    }
    
    /**
     * Sets the setpoint for the arm to use with PID control.
     * 
     * @param armSetpoint the current setpoint which will be updated.
     * @param newArmSetpoint The new setpoint which updates the old one.
     */
    public static void setArmSetpoint(double newArmSetpoint) {
        double armSetpoint = newArmSetpoint;
    }

    /**
     * Sets the maximum amount of power that can be used by the motor(s) on the arm.
     * 
     * @param newArmPowerCoefficient The amount of power (scaled 0 - 1) that will be multiplied by the amp limits on the motors.
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

        //more variables will be needed once encoders are programmed
        return 0.0;
    }
}

