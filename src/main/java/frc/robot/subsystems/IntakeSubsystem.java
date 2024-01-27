package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * manages all motors/encoders/sensors contained in the robot intake
     */
    public IntakeSubsystem() {


    }

    /**
     * Sets the setpoint (RPM) for the arm to use with PID control.
     * 
     * @param newIntakeRMPSetpoint The new setpoint (RPM) which updates the old one.
     */
    public void setIntakeRPMSetpoint(double newIntakeRMPSetpoint) {

        double intakeRPMSetpoint = newIntakeRMPSetpoint;

    }

    /**
     * Sets the max power to be used by the intake motors.
     * 
     * @param newIntakePowerCoefficient The new value which will be multiplied by the amp limits of each motor.
     */
    public void setIntakePower(double newIntakePowerCoefficient) {
        
        double intakePowerCoefficient = newIntakePowerCoefficient;
    }

    /**
     * Returns the intake velocity in RPMs.
     * 
     * @return the intake velocity in RPMs.
     */
    public double intakeRPM() {

        //Will need more variables once encoders are programmed
        return 0.0;
    }
}
