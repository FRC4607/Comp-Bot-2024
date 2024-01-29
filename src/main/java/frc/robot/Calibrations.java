package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;

/**
 * A collection of static classes used to hold calibration values. These values
 * aren't fixed and can be tuned as needed.
 */
public final class Calibrations {
    /**
     * Calibrations for the drivetrain.
     */
    public static final class DrivetrainCalibrations {
        // ** The gains for the steer motors of each swerve module, with gains in amps
        // and rotations. */
        public static final Slot0Configs kSteerGains = new Slot0Configs()
                .withKP(300).withKI(0).withKD(25)
                .withKS(2.885833333).withKV(0).withKA(0);

        /**
         * The gains for the drive motors of each swerve module, with gains in amps and
         * rotations per second.
         */
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(7).withKI(0).withKD(0)
                .withKS(2.447242424).withKV(0).withKA(0);

        /**
         * The maximum current that can be applied to the drive motor of a robot locked
         * in place before the wheels start to slip.
         */
        public static final double kSlipCurrentA = 300;

        /**
         * The speed the robot would attain in meters per second if each of its drive
         * motors had 12V applied to them.
         */
        public static final double kSpeedAt12VoltsMps = Units.feetToMeters(17.3); // From SDS's website for a FOC Falcon
                                                                                  // with L3 gearing
        /** The CANcoder offset of the front left module. */
        public static final double kFrontLeftEncoderOffset = -0.150146484375;
        /** The CANcoder offset of the front right module. */
        public static final double kFrontRightEncoderOffset = -0.048095703125;
        /** The CANcoder offset of the back left module. */
        public static final double kBackLeftEncoderOffset = -0.254150390625;
        /** The CANcoder offset of the back right module. */
        public static final double kBackRightEncoderOffset = -0.037109375;

        // These are only used for simulation
        /** The simulated inertia of the steer motor of a swerve module(?). */
        public static final double kSteerInertia = 0.00001;
        /** The simulated inertia of the drive motor of a swerve module(?). */
        public static final double kDriveInertia = 0.001;
    }

    /**
     * Calibrations for the shooter.
     */
    public static final class ShooterCalibrations {
        /**
         * The kP constant of each of the halves of the shooter. In units of
         * amps/(rotation per second).
         */
        public static final double kP = 12.0;
        /** The kS constant of each of the halves of the shooter. In unit of amps. */
        public static final double kS = 4.875;
    }

    /**
     * Calibrations for the wrist.
     */
    public static final class WristCalibrations {
        /**
         * The kP constant of the wrist motor. In units of (fractional duty
         * cycle)/degree.
         */
        public static final double kP = 0.008333;
        /**
         * The kI constant of the wrist motor. In units of (fractional duty
         * cycle)/degree^2.
         */
        public static final double kI = 0.0;
        /**
         * The kD constant of the wrist motor. In units of (fractional duty
         * cycle)/(degrees per second).
         */
        public static final double kD = 0.0;
        /**
         * The kFF constant of the wrist motor. In units of fractional duty
         * cycle.
         */
        public static final double kFF = 0.0;
    }
}
