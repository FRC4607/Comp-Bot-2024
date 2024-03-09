package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

/**
 * A collection of static classes that hold constant values. These values are not expected to change often.
 */
public final class Constants {
    /** Constants for the drivetrain. */
    public static final class DrivetrainConstants {
        /** The closed-loop output type to use for the steer motors; This affects the PID/FF gains for the steer motors. */
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        /** The closed-loop output type to use for the drive motors; This affects the PID/FF gains for the drive motors. */
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

        /** The ratio of drive motor rotations per azimuth rotation. */
        public static final double kCoupleRatio = 3.5714285714285716;

        /** The ratio of drive motor turns to one wheel rotation. */
        public static final double kDriveGearRatio = 6.122448979591837;
        /** The ratio of steer motor turns to one azimuth rotation. */
        public static final double kSteerGearRatio = 12.8;
        /** The radius of the wheel attached to each swerve module. */
        public static final double kWheelRadiusInches = 2;

        /** Whether or not the steer motors on the robot should be inverted. */
        public static final boolean kSteerMotorReversed = false;
        /** Whether or not the drive motors on the left side of the robot should be inverted. */
        public static final boolean kInvertLeftSide = false;
        /** Whether or not the drive motors on the right side of the robot should be inverted. */
        public static final boolean kInvertRightSide = true;

        /** The name of hCAN bus the drivetrain is on. If attached to the RoboRIO, specify "rio". */
        public static final String kCANbusName = "kachow";
        /** The CAN Id of the Pigeon 2. */
        public static final int kPigeonId = 0;

        // Front Left
        /** The CAN Id of the front left drive motor. */
        public static final int kFrontLeftDriveMotorId = 0;
        /** The CAN Id of the front left steer motor. */
        public static final int kFrontLeftSteerMotorId = 2;
        /** The CAN Id of the front left turn encoder. */
        public static final int kFrontLeftEncoderId = 18;

        /** The X position of the axis of the azimuth of the front left module from the center of the robot. */
        public static final double kFrontLeftXPosInches = 9.75;
        /** The Y position of the axis of the azimuth of the front left module from the center of the robot. */
        public static final double kFrontLeftYPosInches = 9.75;

        // Front Right
        /** The CAN Id of the front right drive motor. */
        public static final int kFrontRightDriveMotorId = 1;
        /** The CAN Id of the front right steer motor. */
        public static final int kFrontRightSteerMotorId = 3;
        /** The CAN Id of the front right turn encoder. */
        public static final int kFrontRightEncoderId = 16;

        /** The X position of the axis of the azimuth of the front right module from the center of the robot. */
        public static final double kFrontRightXPosInches = 9.75;
        /** The Y position of the axis of the azimuth of the front right module from the center of the robot. */
        public static final double kFrontRightYPosInches = -9.75;

        // Back Left
        /** The CAN Id of the back left drive motor. */
        public static final int kBackLeftDriveMotorId = 19;
        /** The CAN Id of the back left steer motor. */
        public static final int kBackLeftSteerMotorId = 17;
        /** The CAN Id of the back left turn encoder. */
        public static final int kBackLeftEncoderId = 3;

        /** The X position of the axis of the azimuth of the back left module from the center of the robot. */
        public static final double kBackLeftXPosInches = -9.75;
        /** The Y position of the axis of the azimuth of the back left module from the center of the robot. */
        public static final double kBackLeftYPosInches = 9.75;

        // Back Right
        /** The CAN Id of the back right drive motor. */
        public static final int kBackRightDriveMotorId = 18;
        /** The CAN Id of the back right steer motor. */
        public static final int kBackRightSteerMotorId = 16;
        /** The CAN Id of the back right turn encoder. */
        public static final int kBackRightEncoderId = 1;

        /** The X position of the axis of the azimuth of the back right module from the center of the robot. */
        public static final double kBackRightXPosInches = -9.75;
        /** The Y position of the axis of the azimuth of the back right module from the center of the robot. */
        public static final double kBackRightYPosInches = -9.75;
    }

    /** Constants for the shooter. */
    public static final class ShooterConstants {
        /** The CAN Id of the upper shooter motor. */
        public static final int kUpperCANId = 5;
        /** The CAN Id of the lower shooter motor. */
        public static final int kLowerCANId = 4;

        /** Whether or not to invert the upper shooter motor. */
        public static final boolean kInvertUpper = true;
        /** Whether or not to invert the lower shooter motor. */
        public static final boolean kInvertLower = false;
    }

    /** Constants for the kicker. */
    public static final class KickerConstants {
        /** The CAN Id of the kicker wheel motor controller. */
        public static final int kCANId = 14;
        /** The gear ratio (x input rotations : 1 output rotation) of the kicker motor to the kicker. */
        public static final double kKickerGearRatio = 5.0;
        /** The diameter of the kickers in mm. */
        public static final double kKickerDiameter = 24.5;
    }

    /** Constants for the intake. */
    public static final class IntakeConstants {
        /** The CAN Id of the roller motor controller. */
        public static final int kRollerCANId = 12;
        /** The CAN Id of the agitator motor controller. */
        public static final int kAgitatorCANId = 9;
        /** Whether or not to invert the roller motor. */
        public static final boolean kRollerInverted = false;
        /** Whether or not to invert the agitator motor. */
        public static final boolean kAgitatorInverted = true;
        /** The gear ratio (x input rotations : 1 output rotation) of the roller motor to the rollers. */
        public static final double kRollerGearRatio = 3.0;
        /** The diameter of the rollers in mm. */
        public static final double kRollerDiameter = 40.5;
        /** The gear ratio (x input rotations : 1 output rotation) of the agitator motor to the agitators. */
        public static final double kAgitatorGearRatio = 1.0;
        /** The diameter of the rollers in mm. */
        public static final double kAgitatorDiameter = 101.6;
    }

    /** Constants for the wrist. */
    public static final class WristConstants {
        /** The CAN Id of the wrist motor controller. */
        public static final int kCANId = 15;
        /** The number of rotor rotations to produce one sensor rotation. */
        public static final double kRotorToSensor = 1.0;
        /** The number of sensor rotations to produce one mechanism rotation. */
        public static final double kSensorToMechanism = 100.0;
        /** In manual control, the top speed of the wrist in degrees per second. */
        public static final double kWristManualDegreesPerSecond = 10.0;
    }

    /** Constants for the arm. */
    public static final class ArmConstants {
        /** The CAN Id of the front (closest to intake) motor in the arm gearbox. */
        public static final int kFrontCANId = 8;
        /** The CAN Id of the rear (closest to battery) motor in the arm gearbox. */
        public static final int kRearCANId = 7;
        /** Whether or not to invert both motors. */
        public static final boolean kInvertMotors = true;
        /** The minimum angle (in rotations) the arm is allowed to go in reverse. SW limit only. */
        public static final double kReverseSoftLimit = 0.0;
        /** The ratio of encoder rotations to wrist rotations. */
        public static final double kSensorToMechanism = 60.0 / 37.0;
        /** The ratio of motor rotations to encoder rotations. */
        public static final double kRotorToSensor = 60.19 / kSensorToMechanism;
        /** Whether or not to invert the encoder on the arm. */
        public static final boolean kInvertEncoder = true;
    }

    /** Constants for the climber. */
    public static final class ClimberConstants {
        /** the CAN Id of the left side climber motor */
        public static final int kLeftCANId = 11;
        /** the can Id of the right side climber motor */
        public static final int kRightCANId = 14;
        /** the amp limit for each motor */
        public static final int kSupplyAmpLimit = 120;
    
    
    
    
    }
}
