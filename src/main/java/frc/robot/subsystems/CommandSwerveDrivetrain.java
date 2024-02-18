package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * Subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private static final SwerveRequest.ApplyChassisSpeeds m_autoSetter = new SwerveRequest.ApplyChassisSpeeds()
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDriveRequestType(DriveRequestType.Velocity);
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private DoubleLogEntry m_logFailDAQ = new DoubleLogEntry(DataLogManager.getLog(), "swerve/daq_fail");
    private StructArrayLogEntry<SwerveModuleState> m_logCurrentState = StructArrayLogEntry
            .create(DataLogManager.getLog(), "swerve/modules", SwerveModuleState.struct);
    private StructArrayLogEntry<SwerveModuleState> m_logTargetState = StructArrayLogEntry
            .create(DataLogManager.getLog(), "swerve/targets", SwerveModuleState.struct);
    private DoubleLogEntry m_logPeriod = new DoubleLogEntry(DataLogManager.getLog(), "swerve/period");
    private StructLogEntry<Pose2d> m_logPose = StructLogEntry.create(DataLogManager.getLog(), "swerve/pose",
            Pose2d.struct);
    private StructLogEntry<ChassisSpeeds> m_logSpeeds = StructLogEntry.create(DataLogManager.getLog(), "swerve/speeds",
            ChassisSpeeds.struct);

    private CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.registerTelemetry(this::telemetry);
    }

    private CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.registerTelemetry(this::telemetry);
    }

    /**
     * Configures PathPlanner's auto builder.
     */
    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> {
                    return this.getState().Pose;
                },
                this::seedFieldRelative,
                () -> {
                    return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
                },
                (speeds) -> {
                    this.setControl(m_autoSetter.withSpeeds(speeds));
                },
                new HolonomicPathFollowerConfig(
                        new PIDConstants(20.0, 0.0, 0.0),
                        new PIDConstants(10.0, 0.0, 0.0),
                        Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps,
                        m_moduleLocations[0].getNorm(),
                        new ReplanningConfig(
                                false,
                                false),
                        1 / this.UpdateFrequency),
                () -> false, // change
                this);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Creates a command that applies a given request every loop.
     * 
     * @param requestSupplier A function that returns the request to apply.
     * @return A command that applies the request returned by the function every
     *         loop.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * A callback that is called by the odometry thread when new data is available.
     * 
     * @param state The
     *              {@link com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState}
     *              object given by the odometry thread.
     */
    private void telemetry(SwerveDriveState state) {
        m_logFailDAQ.append(state.FailedDaqs);
        m_logCurrentState.append(state.ModuleStates);
        m_logTargetState.append(state.ModuleTargets);
        m_logPeriod.append(state.OdometryPeriod);
        m_logPose.append(state.Pose);
        m_logSpeeds.append(state.speeds);
    }

    /**
     * Creates an instance of this class with proper configuration values.
     * 
     * @return An instance of this class that has been properly configured.
     */
    public static CommandSwerveDrivetrain getInstance() {
        SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(Constants.DrivetrainConstants.kPigeonID)
                .withCANbusName(Constants.DrivetrainConstants.kCANbusName);

        SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(Constants.DrivetrainConstants.kDriveGearRatio)
                .withSteerMotorGearRatio(Constants.DrivetrainConstants.kSteerGearRatio)
                .withWheelRadius(Calibrations.DrivetrainCalibrations.kWheelRadiusInches)
                .withSlipCurrent(Calibrations.DrivetrainCalibrations.kSlipCurrentA)
                .withSteerMotorGains(Calibrations.DrivetrainCalibrations.kSteerGains)
                .withDriveMotorGains(Calibrations.DrivetrainCalibrations.kDriveGains)
                .withSteerMotorClosedLoopOutput(Constants.DrivetrainConstants.kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(Constants.DrivetrainConstants.kDriveClosedLoopOutput)
                .withSpeedAt12VoltsMps(Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps)
                .withSteerInertia(Calibrations.DrivetrainCalibrations.kSteerInertia)
                .withDriveInertia(Calibrations.DrivetrainCalibrations.kDriveInertia)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(Constants.DrivetrainConstants.kCoupleRatio)
                .withSteerMotorInverted(Constants.DrivetrainConstants.kSteerMotorReversed);

        SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                Constants.DrivetrainConstants.kFrontLeftSteerMotorID,
                Constants.DrivetrainConstants.kFrontLeftDriveMotorID, Constants.DrivetrainConstants.kFrontLeftEncoderID,
                Calibrations.DrivetrainCalibrations.kFrontLeftEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kFrontLeftXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kFrontLeftYPosInches),
                Constants.DrivetrainConstants.kInvertLeftSide);
        SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                Constants.DrivetrainConstants.kFrontRightSteerMotorID,
                Constants.DrivetrainConstants.kFrontRightDriveMotorID,
                Constants.DrivetrainConstants.kFrontRightEncoderID,
                Calibrations.DrivetrainCalibrations.kFrontRightEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kFrontRightXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kFrontRightYPosInches),
                Constants.DrivetrainConstants.kInvertRightSide);
        SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                Constants.DrivetrainConstants.kBackLeftSteerMotorID,
                Constants.DrivetrainConstants.kBackLeftDriveMotorID, Constants.DrivetrainConstants.kBackLeftEncoderID,
                Calibrations.DrivetrainCalibrations.kBackLeftEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackLeftXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackLeftYPosInches),
                Constants.DrivetrainConstants.kInvertLeftSide);
        SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                Constants.DrivetrainConstants.kBackRightSteerMotorID,
                Constants.DrivetrainConstants.kBackRightDriveMotorID, Constants.DrivetrainConstants.kBackRightEncoderID,
                Calibrations.DrivetrainCalibrations.kBackRightEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackRightXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackRightYPosInches),
                Constants.DrivetrainConstants.kInvertRightSide);

        return new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
