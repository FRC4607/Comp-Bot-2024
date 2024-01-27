package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    /**
     * The subsystem which contains all motors/sensors/encoers on the robot climbers.
     */
    public ClimberSubsystem() {


    }

    /**
     * Sets the setpoint of the left climber to be used by PID control.
     * 
     * @param newLeftClimberSetpoint The new setpoint which will update the old one.
     */
    public void setLeftClimberSetpoint(double newLeftClimberSetpoint) {

        double leftclimbersetpoint = newLeftClimberSetpoint;
    }

    /**
     * Sets the setpoint of the right climber to be used by PID control.
     * 
     * @param newRightClimberSetpoint The new setpoint which will upate the old one.
     */
    public void setRightClimberSetpoint(double newRightClimberSetpoint) {

    }

    /**
     * Sets the power available to be used by the climbers.
     * 
     * @param newClimberPowerCoefficient The new coefficient which will update the old one.
     */
    public void setClimberPower(double newClimberPowerCoefficient) {

        double climberPowerCoefficient = newClimberPowerCoefficient;
    }

    /**
     * gets the position of the left climber
     * 
     * @return left climber length in inches
     */
    public double leftClimberPosition() {

        //will need more variables once encoders are added
        return 0.0;
    }

    /**
     * gets the position of the right climber
     * 
     * @return right climber length in inches
     */
    public double rightClimberPosition() {

        //will need more variables once encoders are added
        return 0.0;
    }
}
