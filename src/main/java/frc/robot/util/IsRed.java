package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class IsRed {
    /**
     * Returns true if the Driver's Station is connect and reports a red alliance.
     * If blue or not connected, returns false.
     * 
     * @return If the robot is definitively on the red alliance.
     */
    public static boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        } else {
            return false;
        }
    }
}
