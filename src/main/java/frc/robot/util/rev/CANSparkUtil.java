package frc.robot.util.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

/**
 * Utilities for working with REV's CAN-enabled brushless motor controllers.
 */
public class CANSparkUtil {
    /**
     * Configures the PIDFF on a giver SPARK Max/Flex.
     * 
     * @param p  The desired P gain.
     * @param i  The desired I gain.
     * @param d  The desired D gain.
     * @param ff The desired FF gain.
     * @param m  The motor to configure the gains on.
     */
    public static void ConfigPIDCANSpark(double p, double i, double d, double ff, CANSparkBase m) {
        SparkPIDController c = m.getPIDController();
        c.setP(p);
        c.setI(i);
        c.setD(d);
        c.setFF(ff);
    }
}
