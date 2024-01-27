package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    /**
     * contains all of the motors/encoders/sensors within the shooter of the robot
     */
    public ShooterSubsystem() {


    }

    /**
     * Sets the RPM setpoint of the shooter to be used with PID control.
     * 
     * @param newShooterRPMsetpoint The new setpoint (RPM) which updates the old one.
     */
    public void setShooterRPMSetpoint(double newShooterRPMSetpoint) {

        double ShooterRPMSetpoint = newShooterRPMSetpoint;

    }
    
    /**
     * Sets the max power that the shooter can use.
     * 
     * @param newShooterPowerCoefficient The new value which will be multiplied by the amp limits of each motor.
     */
    public void setShooterPower(double newShooterPowerCoefficient) {

        double shooterPowerCoefficient = newShooterPowerCoefficient;
    }

    /**
     * Sets the feeder RPM Setpoint to be used by PID control.
     * 
     * @param newFeederRPMSetpoint The new setpoint (RPM) which updates the old one.
     */
    public void setFeederRPMSetpoint(double newFeederRPMSetpoint) {

        double feederRPMSetpoint = newFeederRPMSetpoint;
    }

    /**
     * Returns the current RPMs of the shooter.
     * 
     * @return the current RPMs of the Shooter.
     */
    public double shooterRPM() {

        //will need more variables once encoders are added
        return 0.0;
    }
}
