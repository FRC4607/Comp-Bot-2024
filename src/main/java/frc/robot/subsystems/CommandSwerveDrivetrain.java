package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.Field;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.util.IsRed;
import frc.robot.util.ctre.TalonFXStandardSignalLogger;
import frc.robot.util.som.InterpolatingTreeMapShooter;
import frc.robot.util.som.ShotInfo;
import frc.robot.util.som.ShotInfoWithDirection;

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

    private DoubleLogEntry m_logFailDAQ = new DoubleLogEntry(DataLogManager.getLog(), "/swerve/daqfail");
    private StructArrayLogEntry<SwerveModuleState> m_logCurrentState = StructArrayLogEntry
            .create(DataLogManager.getLog(), "swerve/modules", SwerveModuleState.struct);
    private StructArrayLogEntry<SwerveModuleState> m_logTargetState = StructArrayLogEntry
            .create(DataLogManager.getLog(), "swerve/targets", SwerveModuleState.struct);
    private DoubleLogEntry m_logPeriod = new DoubleLogEntry(DataLogManager.getLog(), "/swerve/period");
    private StructLogEntry<Pose2d> m_logPose = StructLogEntry.create(DataLogManager.getLog(), "/swerve/pose",
            Pose2d.struct);
    private StructLogEntry<ChassisSpeeds> m_logSpeeds = StructLogEntry.create(DataLogManager.getLog(),
            "/swerve/speeds",
            ChassisSpeeds.struct);

    private static final String[] PREFIXES = { "/fl/turn", "/fl/drive", "/fr/turn", "/fr/drive", "/rl/turn",
            "/rl/drive", "/rr/turn", "/rr/drive" };
    private final TalonFXStandardSignalLogger[] m_logs = new TalonFXStandardSignalLogger[8];

    private final int m_kickerSubscriber;
    private final StructLogEntry<Pose2d> m_kickerLog = StructLogEntry.create(DataLogManager.getLog(), "kickerll/mt1",
            Pose2d.struct);
    private final int m_flywheelSubscriber;
    private final StructLogEntry<Pose2d> m_flywheelLog = StructLogEntry.create(DataLogManager.getLog(),
            "flywheelll/mt1",
            Pose2d.struct);
    private final int m_frontSubscriber;
    private final StructLogEntry<Pose2d> m_frontLog = StructLogEntry.create(DataLogManager.getLog(),
            "frontll/mt1",
            Pose2d.struct);

    private final int m_kickerSubscriberMT2;
    private final StructLogEntry<Pose2d> m_kickerLogMT2 = StructLogEntry.create(DataLogManager.getLog(), "kickerll/mt2",
            Pose2d.struct);
    private final int m_flywheelSubscriberMT2;
    private final StructLogEntry<Pose2d> m_flywheelLogMT2 = StructLogEntry.create(DataLogManager.getLog(),
            "flywheelll/mt2",
            Pose2d.struct);
    private final int m_frontSubscriberMT2;
    private final StructLogEntry<Pose2d> m_frontLogMT2 = StructLogEntry.create(DataLogManager.getLog(),
            "frontll/mt2",
            Pose2d.struct);

    private final DoubleArrayPublisher m_kickerPusher = NetworkTableInstance.getDefault().getTable("limelight-kick")
            .getDoubleArrayTopic("robot_orientation_set").publish(PubSubOption.periodic(0.020));
    private final DoubleArrayPublisher m_fwPusher = NetworkTableInstance.getDefault().getTable("limelight-fw")
            .getDoubleArrayTopic("robot_orientation_set").publish(PubSubOption.periodic(0.020));
    private final DoubleArrayPublisher m_frontPusher = NetworkTableInstance.getDefault().getTable("limelight-front")
            .getDoubleArrayTopic("robot_orientation_set").publish(PubSubOption.periodic(0.020));

    private ShotInfoWithDirection m_shotInfo;
    private final InterpolatingTreeMapShooter m_map = InterpolatingTreeMapShooter.getShotMap();
    private Translation2d m_rotationPoint;
    private final DoubleSupplier m_armAngle;
    private final DoubleSupplier m_wristAngle;

    private final TorqueCurrentFOC m_sysidReq = new TorqueCurrentFOC(0);

    private final SysIdRoutine m_turnRoutine = new SysIdRoutine(new SysIdRoutine.Config(
            Volts.of(0.5).per(Seconds.of(1)), // 0.25 "volts" (Amps) per second
            Volts.of(60), // 100 "volts" amps
            Seconds.of(3600), // Long timeout
            (state) -> {
                SignalLogger.writeString("sysid-state", state.toString());
            }),
            new SysIdRoutine.Mechanism((volts) -> {
                try {
                    Field theFunny = this.getModule(0).getClass().getDeclaredField("m_angleTorqueSetter");
                    theFunny.setAccessible(true);
                    MotionMagicTorqueCurrentFOC req = (MotionMagicTorqueCurrentFOC) theFunny.get(this.getModule(0));
                    req.FeedForward = volts.magnitude();
                    this.getModule(0).apply(new SwerveModuleState(), DriveRequestType.Velocity);
                    SignalLogger.writeDouble("sysid-current", volts.magnitude());
                } catch (NoSuchFieldException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (SecurityException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (IllegalArgumentException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }, null, this, "swerve-steer"));
    private final SysIdRoutine m_driveRoutine = new SysIdRoutine(new SysIdRoutine.Config(
            Volts.of(1).per(Seconds.of(1)), // 0.25 "volts" (Amps) per second
            Volts.of(60), // 100 "volts" amps
            Seconds.of(3600), // Long timeout
            (state) -> {
                SignalLogger.writeString("sysid-state", state.toString());
            }),
            new SysIdRoutine.Mechanism((volts) -> {
                for (SwerveModule m : this.Modules) {
                    VelocityTorqueCurrentFOC request;
                    try {
                        Field requestField = m.getClass().getDeclaredField("m_velocityTorqueSetter");
                        requestField.setAccessible(true);
                        request = (VelocityTorqueCurrentFOC) requestField.get(m);
                        request.FeedForward = volts.magnitude();
                        m.apply(new SwerveModuleState(), DriveRequestType.Velocity);
                    } catch (NoSuchFieldException e) {
                        // uh oh
                        System.err.println(e.toString());
                    } catch (IllegalAccessException e) {
                        // uh oh
                        System.err.println(e.toString());
                    }
                }
            }, null, this, "swerve-drive"));

    private CommandSwerveDrivetrain(DoubleSupplier armAngle, DoubleSupplier wristAngle,
            SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();

        }
        this.registerTelemetry(this::telemetry);
        for (int i = 0; i < 4; i++) {
            m_logs[i * 2] = new TalonFXStandardSignalLogger(this.Modules[i].getSteerMotor(),
                    PREFIXES[i * 2], true);
            m_logs[i * 2 + 1] = new TalonFXStandardSignalLogger(this.Modules[i].getDriveMotor(),
                    PREFIXES[i * 2 + 1],
                    true);
        }
        m_kickerSubscriber = NetworkTableInstance.getDefault().getTable("limelight-kick").addListener(
                "botpose_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdate(pose, result[9], result[7], result[6]);
                    m_kickerLog.append(pose);
                });
        m_flywheelSubscriber = NetworkTableInstance.getDefault().getTable("limelight-fw").addListener(
                "botpose_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdate(pose, result[9], result[7], result[6]);
                    m_flywheelLog.append(pose);
                });
        m_frontSubscriber = NetworkTableInstance.getDefault().getTable("limelight-front").addListener(
                "botpose_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdate(pose, result[9], result[7], result[6]);
                    m_frontLog.append(pose);
                });

        m_kickerSubscriberMT2 = NetworkTableInstance.getDefault().getTable("limelight-kick").addListener(
                "botpose_orb_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdateMT2(pose, result[9], result[7], result[6]);
                    m_kickerLogMT2.append(pose);
                });
        m_flywheelSubscriberMT2 = NetworkTableInstance.getDefault().getTable("limelight-fw").addListener(
                "botpose_orb_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdateMT2(pose, result[9], result[7], result[6]);
                    m_flywheelLogMT2.append(pose);
                });
        m_frontSubscriberMT2 = NetworkTableInstance.getDefault().getTable("limelight-front").addListener(
                "botpose_orb_wpiblue",
                EnumSet.of(Kind.kValueRemote),
                (NetworkTable table, String topic, NetworkTableEvent event) -> {
                    double[] result = event.valueData.value.getDoubleArray();
                    Pose2d pose = new Pose2d(result[0], result[1],
                            Rotation2d.fromDegrees(result[5]));
                    this.handleLLUpdateMT2(pose, result[9], result[7], result[6]);
                    m_frontLogMT2.append(pose);
                });

        m_armAngle = armAngle;
        m_wristAngle = wristAngle;
    }

    private CommandSwerveDrivetrain(DoubleSupplier armAngle, DoubleSupplier wristAngle,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        this(armAngle, wristAngle, driveTrainConstants, 250, modules);
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
                        new PIDConstants(Calibrations.DrivetrainCalibrations.kPPathPlannerTranslation, 0.0, 0.0),
                        new PIDConstants(Calibrations.DrivetrainCalibrations.kPPathPlannerRotation, 0.0, 0.0),
                        Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps,
                        m_moduleLocations[0].getNorm(),
                        new ReplanningConfig(
                                false,
                                false),
                        1 / this.UpdateFrequency),
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red;
                    } else {
                        return false;
                    }
                },
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
        double data[] = new double[] { state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0 };
        m_kickerPusher.set(data);
        m_fwPusher.set(data);
        m_frontPusher.set(data);
        m_logFailDAQ.append(state.FailedDaqs);
        m_logCurrentState.append(state.ModuleStates);
        m_logTargetState.append(state.ModuleTargets);
        m_logPeriod.append(state.OdometryPeriod);
        m_logPose.append(state.Pose);
        m_logSpeeds.append(state.speeds);
    }

    private void handleLLUpdate(Pose2d pose, double distance, double tags, double latency) {
        // Make sure we are using good data.
        if (tags > 0 && (DriverStation.isDisabled()
                || (tags > 1 && distance < 3))) {
            // Add the vision measurement. The standard deviation for the rotation is very
            // high as general advice is to not trust it.
            this.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (latency / 1000.0),
                    VecBuilder.fill(999999999, 999999999, Math.PI * 2));
        }
    }

    private void handleLLUpdateMT2(Pose2d pose, double distance, double tags, double latency) {
        // Make sure we are using good data.
        if (tags > 0 && !DriverStation.isDisabled()) {
            // First, we compute estimated standard deviations for the X and Y measurements.
            // The approach is borrowed from 6328, see here for more info:
            // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/292.
            double stdDevXY = (0.7 + 0.07 * Math.pow(distance, 2)) / Math.pow(tags, 2);
            // Add the vision measurement. The standard deviation for the rotation is very
            // high as general advice is to not trust it.
            this.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (latency / 1000.0),
                    VecBuilder.fill(stdDevXY, stdDevXY, 999999999));
        }
    }

    /**
     * Returns the current offset used for field-relative driving.
     * 
     * @return The robot's current field relative offset as an
     *         {@link edu.wpi.first.math.geometry.Rotation2d}.
     */
    public Rotation2d getSwerveOffset() {
        return m_fieldRelativeOffset;
    }

    public Translation2d getRotationPoint() {
        return m_rotationPoint;
    }

    public ShotInfoWithDirection getShotInfo() {
        return m_shotInfo;
    }

    @Override
    public void periodic() {
        for (TalonFXStandardSignalLogger log : m_logs) {
            log.log();
        }

        // Parts of this shoot on the move code be plundered from 7028
        // https://github.com/STMARobotics/frc-7028-2024/blob/d9b8a288764409806fc8b7e71752e7c628bdd833/src/main/java/frc/robot/commands/ScoreSpeakerTeleopCommand.java

        // Get stationary position
        double shooterOffset = Constants.ArmConstants.kArmOffsetX
                + Math.cos(Math.toRadians(m_armAngle.getAsDouble())) * Constants.ArmConstants.kArmLength
                + Math.cos(Math.toRadians(m_wristAngle.getAsDouble()))
                        * Constants.WristConstants.kWristEffectiveLength;
        m_rotationPoint = new Translation2d(shooterOffset, 0.0);
        Pose2d compensatedRobotPose = new Pose2d(this.m_cachedState.Pose.getTranslation().plus(
                m_rotationPoint.rotateBy(m_cachedState.Pose.getRotation())), this.m_cachedState.Pose.getRotation());
        Translation2d speakerPos = IsRed.isRed() ? Constants.DrivetrainConstants.kRedAllianceSpeakerPosition
                : Constants.DrivetrainConstants.kBlueAllianceSpeakerPosition;
        double dist1 = speakerPos.getDistance(compensatedRobotPose.getTranslation());
        SmartDashboard.putNumber("Robot Distance", dist1);
        ShotInfo step1 = m_map.get(Math.min(5.94, Math.max(1.369, dist1)));
        ChassisSpeeds fr = ChassisSpeeds.fromRobotRelativeSpeeds(this.m_cachedState.speeds,
                compensatedRobotPose.getRotation());

        // Find second compensated position
        double scoreTime = SmartDashboard.getNumber("SOM", Calibrations.DrivetrainCalibrations.kShootOnMoveConstant)
                * (dist1 / (step1.getSpeed() * Math.abs(Math.cos(Math.toRadians(step1.getWrist()))))); // Multiply by
                                                                                                       // cosine of
                                                                                                       // wrist to get X
                                                                                                       // velocity
        Translation2d movementOffset = new Translation2d(fr.vxMetersPerSecond * scoreTime,
                fr.vyMetersPerSecond * scoreTime);
        Translation2d offsetSpeaker = speakerPos.minus(movementOffset);
        double dist2 = offsetSpeaker.getDistance(compensatedRobotPose.getTranslation());
        ShotInfo step2 = m_map
                .get(Math.min(5.94, Math.max(1.369, offsetSpeaker.getDistance(compensatedRobotPose.getTranslation()))));

        // Find third position, hopefully reduicing error
        scoreTime = SmartDashboard.getNumber("SOM", Calibrations.DrivetrainCalibrations.kShootOnMoveConstant)
                * (dist2 / (step2.getSpeed() * Math.abs(Math.cos(Math.toRadians(step2.getWrist()))))); // Multiply by
                                                                                                       // cosine of
                                                                                                       // wrist to get X
                                                                                                       // velocity
        movementOffset = new Translation2d(fr.vxMetersPerSecond * scoreTime,
                fr.vyMetersPerSecond * scoreTime);
        offsetSpeaker = speakerPos.minus(movementOffset);
        ShotInfoWithDirection temp = m_map
                .get(Math.min(5.94, Math.max(1.369, offsetSpeaker.getDistance(compensatedRobotPose.getTranslation()))))
                .withDirection(
                        offsetSpeaker.minus(compensatedRobotPose.getTranslation()).getAngle());
        m_shotInfo = new ShotInfoWithDirection(temp.getSpeed(),
                temp.getWrist() + SmartDashboard.getNumber("SOM Bump", 0.0), temp.getRobot());
    }

    public double[] getWheelRadiusCharacterizationPosition() {
        double[] results = new double[4];
        for (int i = 0; i < 4; i++) {
            results[i] = this.m_logs[1 + 2 * i].m_pos.getValueAsDouble() * 2 * Math.PI
                    / Constants.DrivetrainConstants.kDriveGearRatio;
        }
        return results;
    }

    public Command getTurnQuasistaic(SysIdRoutine.Direction d) {
        return m_turnRoutine.quasistatic(d).onlyWhile(() -> Math.abs(m_logs[0].m_velocity.getValueAsDouble()) <= 7.0);
    }

    public Command getTurnDynamic(SysIdRoutine.Direction d) {
        return m_turnRoutine.dynamic(d).onlyWhile(() -> Math.abs(m_logs[0].m_velocity.getValueAsDouble()) <= 7.0);
    }

    public Command getDriveQuasistatic(SysIdRoutine.Direction d) {
        return m_driveRoutine.quasistatic(d).onlyWhile(() -> Math.abs(m_logs[1].m_velocity.getValueAsDouble()) <= 80.0);
    }

    public Command getDriveDynamic(SysIdRoutine.Direction d) {
        return m_driveRoutine.dynamic(d).onlyWhile(() -> Math.abs(m_logs[1].m_velocity.getValueAsDouble()) <= 80.0);
    }

    /**
     * Creates an instance of this class with proper configuration values.
     * 
     * @return An instance of this class that has been properly configured.
     */
    public static CommandSwerveDrivetrain getInstance(DoubleSupplier armAngle, DoubleSupplier wristAngle) {
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
                Constants.DrivetrainConstants.kFrontLeftDriveMotorID,
                Constants.DrivetrainConstants.kFrontLeftEncoderID,
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
                Constants.DrivetrainConstants.kBackLeftDriveMotorID,
                Constants.DrivetrainConstants.kBackLeftEncoderID,
                Calibrations.DrivetrainCalibrations.kBackLeftEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackLeftXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackLeftYPosInches),
                Constants.DrivetrainConstants.kInvertLeftSide);
        SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                Constants.DrivetrainConstants.kBackRightSteerMotorID,
                Constants.DrivetrainConstants.kBackRightDriveMotorID,
                Constants.DrivetrainConstants.kBackRightEncoderID,
                Calibrations.DrivetrainCalibrations.kBackRightEncoderOffset,
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackRightXPosInches),
                Units.inchesToMeters(Constants.DrivetrainConstants.kBackRightYPosInches),
                Constants.DrivetrainConstants.kInvertRightSide);

        return new CommandSwerveDrivetrain(armAngle, wristAngle, DrivetrainConstants, FrontLeft, FrontRight,
                BackLeft,
                BackRight);
    }
}
