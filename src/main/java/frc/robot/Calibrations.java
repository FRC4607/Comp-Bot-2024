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
                .withKP(500).withKI(0).withKD(25)
                .withKS(10).withKV(0).withKA(0);

        /**
         * The gains for the drive motors of each swerve module, with gains in amps and
         * rotations per second.
         */
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(7).withKI(0).withKD(0)
                .withKS(7.75).withKV(0).withKA(0);

        /**
         * The maximum current that can be applied to the drive motor of a robot locked
         * in place before the wheels start to slip.
         */
        public static final double kSlipCurrentA = 50;

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
        public static final double kP = 0.01;
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
    /** Calibrations for the arm. */
    public static final class ArmCalibrations {
        /** The gravity constant for arm feedforward in Amps. */
        public static final double kG = 25.0;
        /** The static constant for arm feedforward in Amps. */
        public static final double kS = 0.4375;
        /** The proportional constant for arm feedback in A/rot. */
        public static final double kP = 1400.0;
        /** The derivative constant for arm feedback in A/rot*sec. */
        public static final double kD = 200.0;
        /** The maximum speed the arm should acheieve in rot/sec.  */
        public static final double kMaxSpeedMotionMagic = 0.5;
        /** The maximum acceleration the arm should achieve in rot/sec^2. */
        public static final double kMaxAccelerationMotionMagic = 0.5;
        /** The offset of the absolute encoder on the arm in rotations. */
        public static final double kEncoderOffset = -0.75341796875;
        /** The maximum amount of current each motor in the arm gearbox should be allowed to output. */
        public static final double kMaxArmCurrentPerMotor = 40.0;;
    }
    /** Calibrations for the intake. */
    public static final class IntakeCalibrations {
        /** The proportional constant for roller feedback in duty cycle / (mm/s) */
        public static final double kRollerP = 0.0002;
        /** The proportional constant for roller feedback in duty cycle / (mm/s) */
        public static final double kRollerFF = 0.00024;
        /** The proportional constant for agitator feedback in duty cycle / (mm/s) */
        public static final double kAgitatorP = 0.01;
    }
    /** Calibrations for the kicker. */
    public static final class KickerCalibrations {
        /** The proportional constant for roller feedback in duty cycle / (mm/s) */
        public static final double kP = 0.0003;
        /** The derivative constant for roller feedback in duty cycle / (mm/s^2) */
        public static final double kD = 0.005;
        /** The roller feedforward constant in duty cycle / (mm/s) */
        public static final double kFF = 0.0007;
    }
}
