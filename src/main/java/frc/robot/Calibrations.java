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
        /**
         * The gains for the steer motors of each swerve module, with gains in amps
         * and rotations.
         */
        public static final Slot0Configs kSteerGains = new Slot0Configs()
                .withKP(1500).withKI(0).withKD(100)
                .withKS(20).withKV(0.0040532).withKA(0.38524);

        /**
         * The gains for the drive motors of each swerve module, with gains in amps and
         * rotations per second.
         */
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(10).withKI(0).withKD(0)
                .withKS(10.794).withKV(0.044606).withKA(0.60534);

        /**
         * The maximum current that can be applied to the drive motor of a robot locked
         * in place before the wheels start to slip.
         */
        public static final double kSlipCurrentA = 123.5;

        /** The effective radius of the wheel attached to each swerve module. */
        public static final double kWheelRadiusInches = 1.938152547065645; // 3/16/24

        /**
         * The speed the robot would attain in meters per second if each of its drive
         * motors had 12V applied to them.
         */
        public static final double kSpeedAt12VoltsMps = 91.0 / Constants.DrivetrainConstants.kDriveGearRatio
                * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)); // Calculates the robot's free speed from a
                                                                            // max motor speed of 91 rps.

        /** The CANcoder offset of the front left module. */
        public static final double kFrontLeftEncoderOffset = -0.186767578125;
        /** The CANcoder offset of the front right module. */
        public static final double kFrontRightEncoderOffset = -0.08984375;
        /** The CANcoder offset of the back left module. */
        public static final double kBackLeftEncoderOffset = -0.353271484375;
        /** The CANcoder offset of the back right module. */
        public static final double kBackRightEncoderOffset = -0.875732421875;

        // These are only used for simulation
        /** The simulated inertia of the steer motor of a swerve module(?). */
        public static final double kSteerInertia = 0.00001;
        /** The simulated inertia of the drive motor of a swerve module(?). */
        public static final double kDriveInertia = 0.001;

        /**
         * The P value of the PID controler used to control the robot's heading when
         * tracking the speaker.
         */
        public static final double kHeadingPIDP = 16.0;

        /**
         * The I value of the PID controler used to control the robot's heading when
         * tracking the speaker.
         */
        public static final double kHeadingPIDI = 0;

        /**
         * The D value of the PID controler used to control the robot's heading when
         * tracking the speaker.
         */
        public static final double kHeadingPIDD = 0;

        /**
         * A constant that is used during the shoot on the move calculations. Converts
         * between meters/rpm and seconds
         */
        public static final double kShootOnMoveConstant = 300;

        /**
         * The P value of the PID controller that PathPlanner uses to correct for
         * translation error.
         */
        public static final double kPPathPlannerTranslation = 20.0;

        /**
         * The P value of the PID controller that PathPlanner uses to correct for
         * heading error.
         */
        public static final double kPPathPlannerRotation = 10.0;
    }

    /**
     * Calibrations for the shooter.
     */
    public static final class ShooterCalibrations {
        /**
         * The kP constant of the shooter, in Amps/(rotations per second).
         */
        public static final double kP = 12.0;
        /** The kS constant of the shooter in Amps. */
        public static final double kS = 8.0;
        /** The kV constant of the shooter in Amps/(rotations per second) */
        public static final double kV = 0.01;
        /**
         * The maximum current that the velocity PID should output in either direction
         * in Amps.
         */
        public static final double kMaxCurrent = 800.0;
    }

    /**
     * Calibrations for the wrist.
     */
    public static final class WristCalibrations {
        /**
         * The kP constant of the wrist motor. In units of Amps/rotation.
         */
        public static final double kP = 1200.0;
        /**
         * The kI constant of the wrist motor. In units of Amps/rotation^2.
         */
        public static final double kI = 0;
        /**
         * The kD constant of the wrist motor. In units of Amps/rps.
         */
        public static final double kD = 50.0;
        /** The absolute encoder offset of the wrist in rotations. */
        public static final double kEncoderOffset = -0.146240234375;
        /** The kS constant of the wrist in Amps. */
        public static final double kS = 0.0;
        /** The maximum velocity motion magic should use in rps. */
        public static final double kMotionMagicMaxVelocity = 2.0;
        /** The maximum acceleration motion magic should use in rps^2. */
        public static final double kMotionMagicMaxAcceleration = 2.75;
        /** The kG constant of the wrist when the arm is at 0deg, in Amps. */
        public static final double kG = 15.0;
    }

    /** Calibrations for the arm. */
    public static final class ArmCalibrations {
        /** The gravity constant for arm feedforward in Amps. */
        public static final double kG = 20.0;
        /** The static constant for arm feedforward in Amps. */
        public static final double kS = 0.0;
        /** The proportional constant for arm feedback in A/rot. */
        public static final double kP = 400.0;
        /** The derivative constant for arm feedback in A/rot*sec. */
        public static final double kD = 75.0;
        /** The maximum speed the arm should acheieve in rot/sec. */
        public static final double kMaxSpeedMotionMagic = 0.3;
        /** The maximum acceleration the arm should achieve in rot/sec^2. */
        public static final double kMaxAccelerationMotionMagic = 0.75;
        /** The offset of the absolute encoder on the arm in rotations. */
        public static final double kEncoderOffset = -0.5111762152;
        /**
         * The maximum amount of current the controller should be allowed to output in
         * either direction in Amps.
         */
        public static final double kMaxArmCurrentPerMotor = 40.0;
    }

    /** Calibrations for the intake. */
    public static final class IntakeCalibrations {
        /** The proportional constant for roller feedback in Amps/rps. */
        public static final double kRollerP = 10;
        /** The proportional constant for agitator feedback in duty cycle / (mm/s). */
        public static final double kAgitatorP = 0;
        /** The static constant for roller feedforward in Amps. */
        public static final double kRollerS = 9;
        /** The velocity constant for roller feedforward in Amps/(rotation/second) */
        public static final double kRollerV = 0.1;
        /**
         * The derivative constant for roller feedback in Amps/(rotations per second^2).
         */
        public static final double kRollerD = 0;
        /** The maximum current the roller motor should output, in Amps. */
        public static final int kRollerMaxCurrent = 800;
    }

    /** Calibrations for the kicker. */
    public static final class KickerCalibrations {
        /** The proportional constant for roller feedback in Amps/rps. */
        public static final double kP = 10;
        /** The derivative constant for roller feedback in Amps/rps^2. */
        public static final double kD = 0;
        /** The roller static feedforward constant in Amps. */
        public static final double kS = 23.5;
        /** The roller velocity feedforward constant in Amps/(rotation per second) */
        public static final double kV = 0.1;
    }
}
