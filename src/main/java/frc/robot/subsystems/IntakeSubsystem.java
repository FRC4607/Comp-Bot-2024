package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * manages all motors/encoders/sensors contained in the robot intake
     */
    public IntakeSubsystem() {


    }

    /**
     * Sets the maximum power to be used by the intake
     * 
     * @param newIntakePowerCoefficient the new value which upates the old one
     */
    public void setIntakePower(double newIntakePowerCoefficient) {
        
        double intakePowerCoefficient = newIntakePowerCoefficient;
    }
}
