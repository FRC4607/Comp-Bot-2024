package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    /**
     * contains all of the motors/encoders/sensors within the shooter of the robot
     */
    public ShooterSubsystem() {


    }

    /**
     * Sets the RPM setpoint of the shooter.
     * 
     * @param newShooterRPMsetpoint The value which updates the old one.
     */
    public void setShooterRPMSetpoint(double newShooterRPMSetpoint) {

        double ShooterRPMSetpoint = newShooterRPMSetpoint;

    }
    
    /**
     * Sets the max power that the shooter can use.
     * 
     * @param newShooterPowerCoefficient The new value which will update the old one.
     */
    public void setShooterPower(double newShooterPowerCoefficient) {

        double shooterPowerCoefficient = newShooterPowerCoefficient;
    }

    /**
     * Sets the feeder RPM Setpoint.
     * 
     * @param newFeederRPMSetpoint The new RPM value that will update the old one.
     */
    public void setFeederRPMSetpoint(double newFeederRPMSetpoint) {

        double feederRPMSetpoint = newFeederRPMSetpoint;
    }

    /**
     * Gets the RPMs of the shooter.
     * 
     * @return the current RPMs of the Shooter.
     */
    public double shooterRPM() {

        //will need more variables once encoders are added
        return 0.0;
    }
}
