// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CenterNote;
import frc.robot.commands.Climb;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeSync;
import frc.robot.commands.RunKickerWheel;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.ShootOverDefense;
import frc.robot.commands.ShootUsingInterpolation;
import frc.robot.commands.ShootUsingInterpolationWithCorrection;
import frc.robot.commands.SourcePass;
import frc.robot.commands.SourcePassOver;
import frc.robot.commands.SourcePickup;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.IsRed;
import frc.robot.util.swerve.SetCurrentRequest;

public class RobotContainer {

    private static final double MaxSpeed = Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps;
    private static final double MaxAngularRate = Math.PI * 2;

    private static final Rotation2d HALF_ROTATION = Rotation2d.fromDegrees(180);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);
    private final SwerveRequest.FieldCentricFacingAngle autoPoint = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);
    private final SetCurrentRequest current = new SetCurrentRequest();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final WristSubsystem m_wrist = new WristSubsystem(m_arm::armPosition);
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(m_arm::armPosition,
            m_wrist::getWristPosition);
    private final KickerSubsystem m_kicker = new KickerSubsystem();
    private final LEDSubsystem m_leds = new LEDSubsystem();

    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final SendableChooser<Command> m_autoChooser;

    private void configureBindings() {
        m_kicker.setDefaultCommand(new RunIntakeSync(() -> {
            return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
        }, m_intake, m_kicker, m_leds));
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                            .withRotationalRate(
                                    -joystick.getRightX() * MaxAngularRate)
                            .withDeadband(0.1 * MaxSpeed)
                            .withRotationalDeadband(0.1 * MaxAngularRate);
                }));

        // m_shooter.setDefaultCommand(
        // new SetShooterSpeed(() -> SmartDashboard.getNumber("Shooter RPM", 0.0), 120,
        // m_shooter));
        // m_wrist.setDefaultCommand(
        // new MoveWristToPosition(() -> SmartDashboard.getNumber("Wrist Angle Setter",
        // 0.0), 5.0, m_wrist, true));

        // drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> current));
        joystick.a().onTrue(new SetShooterSpeed(() -> 5200, 120, m_shooter))
                .onFalse(new SetShooterSpeed(() -> 0, 120, m_shooter));
        joystick.b().onTrue(new ParallelCommandGroup(
                new SetShooterSpeed(() -> 0, 120, m_shooter),
                new Retract(m_wrist, m_arm)));
        joystick.y().onTrue(new InstantCommand(drivetrain::seedFieldRelative, drivetrain));
        joystick.x().onTrue(new InstantCommand(LEDSubsystem::setIntake).andThen(new RunIntakeSync(() -> {
            return 1;
        }, m_intake, m_kicker, m_leds).andThen(new InstantCommand(LEDSubsystem::setNeutral))).withTimeout(4));
        joystick.povLeft().onTrue(new ParallelCommandGroup(new MoveArmToPosition(() -> 90.0, 7.5, m_arm),
                new MoveWristToPosition(() -> 40.0, 7.5, m_wrist),
                new InstantCommand(LEDSubsystem::setAmp)));

        joystick.povRight().onTrue(new ParallelCommandGroup(
                new MoveArmToPosition(() -> SmartDashboard.getNumber("Trap Shoulder Position", 85.0),
                        7.5, m_arm),
                new MoveWristToPosition(() -> SmartDashboard.getNumber("Trap Wrist Position", 105.0),
                        7.5, m_wrist),
                new InstantCommand(LEDSubsystem::setAmp)));

        joystick.povDown().onTrue(new ParallelCommandGroup(
                new InstantCommand( LEDSubsystem::setShootReady),
                new MoveArmToPosition(() -> 0.0, 7.5, m_arm).andThen(new InstantCommand(() -> {
                    m_arm.setNeutral();
                }, m_arm)),
                new MoveWristToPosition(() -> {
                    return SmartDashboard.getNumber("Wrist Angle Setter", 0.0) == 0.0
                            ? drivetrain.getShotInfo().getWrist()
                            : SmartDashboard.getNumber("Wrist Angle Setter", 0.0);
                }, 10, m_wrist, true),
                new SetShooterSpeed(() -> {
                    return SmartDashboard.getNumber("Shooter RPM", 0.0) == 0.0
                            ? drivetrain.getShotInfo().getSpeed()
                            : SmartDashboard.getNumber("Shooter RPM", 0.0);
                }, 120, m_shooter),
                drivetrain.applyRequest(() -> {
                    if (Math.abs(joystick.getRightX()) > 0.1) {
                        return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.25)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.25)
                                .withDeadband(0.1 * MaxSpeed * 0.25)
                                .withRotationalDeadband(0.1 * MaxAngularRate)
                                .withRotationalRate(
                                        -joystick.getRightX() * MaxAngularRate);
                    } else {
                        Translation2d targetPose = IsRed.isRed()
                                ? Constants.DrivetrainConstants.kRedAllianceSpeakerPosition
                                : Constants.DrivetrainConstants.kBlueAllianceSpeakerPosition;
                        // https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972/3
                        return autoPoint
                                .withTargetDirection(
                                        drivetrain.getShotInfo().getRobot()
                                                .plus(HALF_ROTATION)
                                                .minus(drivetrain
                                                        .getSwerveOffset())
                                                .plus(Rotation2d.fromDegrees(
                                                        SmartDashboard.getNumber(
                                                                "Robot Heading Offset",
                                                                0.0))))
                                .withCenterOfRotation(drivetrain.getRotationPoint())
                                .withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.25)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.25)
                                .withDeadband(0.1 * MaxSpeed * 0.25)
                                .withRotationalDeadband(0.15 * MaxAngularRate * 0.25);
                    }
                })))
                .onFalse(new ParallelDeadlineGroup(
                        new RunKickerWheel(3000.0, m_kicker).withTimeout(1.0),
                        new InstantCommand( LEDSubsystem::setNeutral),
                        drivetrain.applyRequest(() -> autoPoint
                                .withTargetDirection(
                                        drivetrain.getShotInfo().getRobot()
                                                .plus(HALF_ROTATION)
                                                .minus(drivetrain
                                                        .getSwerveOffset())
                                                .plus(Rotation2d.fromDegrees(
                                                        SmartDashboard.getNumber(
                                                                "Robot Heading Offset",
                                                                0.0))))
                                .withCenterOfRotation(drivetrain.getRotationPoint())
                                .withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.25)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.25)
                                .withRotationalDeadband(0.15 * MaxAngularRate * 0.25)))
                        .andThen(new ParallelDeadlineGroup(
                                new ParallelCommandGroup(
                                        new SetShooterSpeed(() -> 0.0, 120,
                                                m_shooter),
                                        new Retract(m_wrist, m_arm)),
                                drivetrain.applyRequest(() -> {
                                    return drive.withVelocityX(
                                            -joystick.getLeftY() * MaxSpeed)
                                            .withVelocityY(-joystick
                                                    .getLeftX()
                                                    * MaxSpeed)
                                            .withRotationalRate(
                                                    -joystick.getRightX()
                                                            * MaxAngularRate)
                                            .withDeadband(0.1 * MaxSpeed)
                                            .withRotationalDeadband(0.1
                                                    * MaxAngularRate);
                                }))));
        joystick.povUp().whileTrue(new ShootOverDefense(joystick::getLeftX, joystick::getLeftY,
                joystick::getRightX, m_arm, m_wrist, m_kicker, m_shooter, drivetrain))
                .onFalse(new ParallelCommandGroup(
                        new SetShooterSpeed(() -> 0.0, 120, m_shooter),
                        new Retract(m_wrist, m_arm)));
        joystick.leftBumper().whileTrue(new SourcePass(m_arm, m_wrist, m_shooter))
                .onFalse(new RunKickerWheel(3000.0, m_kicker).withTimeout(1.0)
                        .andThen(new SetShooterSpeed(() -> 0.0, 10000, m_shooter)));
        joystick.rightBumper().whileTrue(new ParallelCommandGroup(new SourcePassOver(m_arm, m_wrist, m_shooter),
                drivetrain.applyRequest(() -> autoPoint
                        .withTargetDirection(
                                (IsRed.isRed() ? Constants.DrivetrainConstants.kRedAllianceAmpCornerPosition
                                        : Constants.DrivetrainConstants.kBlueAllianceAmpCornerPosition)
                                        .minus(drivetrain.getState().Pose.getTranslation()).getAngle()
                                        .plus(HALF_ROTATION)
                                        .minus(drivetrain
                                                .getSwerveOffset())
                                        .plus(Rotation2d.fromDegrees(
                                                SmartDashboard.getNumber(
                                                        "Robot Heading Offset",
                                                        0.0))))
                        .withCenterOfRotation(drivetrain.getRotationPoint())
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.25)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.25)
                        .withRotationalDeadband(0.15 * MaxAngularRate * 0.25))))
                .onFalse(new RunKickerWheel(3000.0, m_kicker).withTimeout(1.0)
                        .andThen(new SetShooterSpeed(() -> 0.0, 10000, m_shooter)));

        operatorJoystick.povDown().onTrue(new Climb(0, m_climber));
        operatorJoystick.povRight().onTrue(new Climb(38.25, m_climber));
        operatorJoystick.povUp().onTrue(new Climb(85, m_climber));
        operatorJoystick.povLeft().onTrue(new SourcePickup(m_arm, m_wrist));
        operatorJoystick.a().onTrue(new SetShooterSpeed(() -> -3000, 120, m_shooter))
                .onFalse(new SetShooterSpeed(() -> 0, 120, m_shooter));
        operatorJoystick.b().onTrue(new SetShooterSpeed(() -> 350, 120, m_shooter))
                .onFalse(new SetShooterSpeed(() -> 0, 120, m_shooter));
        operatorJoystick.leftStick().onTrue(new InstantCommand(() -> {
            drivetrain.seedFieldRelative(new Pose2d(
                    new Translation2d(
                            IsRed.isRed() ? 16.42 - 0.40005 : 0.40005,
                            8.18 - 0.40005),
                    Rotation2d.fromDegrees(
                            IsRed.isRed() ? 180.0 : 0.0)));
        }, drivetrain));
        operatorJoystick.x().onTrue(new SequentialCommandGroup( // Sides
                new SetShooterSpeed(() -> 4000, 120, m_shooter),
                new MoveWristToPosition(() -> 125.0, 3.0, m_wrist, true),
                new RunKickerWheel(3000.0, m_kicker).withTimeout(0.5),
                new Retract(m_wrist, m_arm),
                new SetShooterSpeed(() -> 0, 120, m_shooter)));
        operatorJoystick.y().onTrue(new SequentialCommandGroup( // Center
                new SetShooterSpeed(() -> 2600, 120, m_shooter),
                new MoveWristToPosition(() -> 125.0, 3.0, m_wrist, true),
                new RunKickerWheel(3000.0, m_kicker).withTimeout(0.5),
                new Retract(m_wrist, m_arm),
                new SetShooterSpeed(() -> 0, 120.0, m_shooter)));
    }

    public RobotContainer() {

        SmartDashboard.putNumber("Trap Wrist Position", 105.0);
        SmartDashboard.putNumber("Trap Shoulder Position", 85.0);
        // SmartDashboard.putNumber("Set Current Request", 0.0);
        SmartDashboard.putNumber("Shooter RPM", 0.0);
        SmartDashboard.putNumber("Wrist Angle Setter", 0.0);
        SmartDashboard.putNumber("SOM", Calibrations.DrivetrainCalibrations.kShootOnMoveConstant);
        SmartDashboard.putNumber("SOM Bump", 0.0);
        SmartDashboard.putNumber("Robot Heading Offset", -4.0);
        SmartDashboard.putData("Run Wheel Radius Test", new WheelRadiusCharacterization(drivetrain));

        // SmartDashboard.putData("Turn QF",
        // drivetrain.getTurnQuasistaic(Direction.kForward));
        // SmartDashboard.putData("Turn QR",
        // drivetrain.getTurnQuasistaic(Direction.kReverse));
        // SmartDashboard.putData("Turn DF",
        // drivetrain.getTurnDynamic(Direction.kForward));
        // SmartDashboard.putData("Turn DR",
        // drivetrain.getTurnDynamic(Direction.kReverse));

        // SmartDashboard.putData("Drive QF",
        // drivetrain.getDriveQuasistatic(Direction.kForward));
        // SmartDashboard.putData("Drive QR",
        // drivetrain.getDriveQuasistatic(Direction.kReverse));
        // SmartDashboard.putData("Drive DF",
        // drivetrain.getDriveDynamic(Direction.kForward));
        // SmartDashboard.putData("Drive DR",
        // drivetrain.getDriveDynamic(Direction.kReverse));

        autoPoint.HeadingController.setPID(
                Calibrations.DrivetrainCalibrations.kHeadingPIDP,
                Calibrations.DrivetrainCalibrations.kHeadingPIDI,
                Calibrations.DrivetrainCalibrations.kHeadingPIDD);
        autoPoint.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        autoPoint.ForwardReference = ForwardReference.RedAlliance;
        configureBindings();
        // Register all of the commands for autos, then set up auto builder and the auto
        // chooser.
        NamedCommands.registerCommand("SetShooterSpeed 5000", new SetShooterSpeed(() -> 5000, 120, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 4000", new SetShooterSpeed(() -> 4000, 60, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed SW Center",
                new SetShooterSpeed(() -> 2600, 60, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 1500", new SetShooterSpeed(() -> 1500, 120, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 0", new SetShooterSpeed(() -> 0, 120, m_shooter));
        NamedCommands.registerCommand("SetArmPosition 36", new MoveArmToPosition(() -> 36, 3, m_arm));
        NamedCommands.registerCommand("SetWristPosition 45",
                new MoveWristToPosition(() -> 125.0, 3.0,
                        m_wrist, true));
        NamedCommands.registerCommand("SetWristPosition 81", new MoveWristToPosition(() -> 148.5, 3, m_wrist));
        NamedCommands.registerCommand("SetWristPosition 76", new MoveWristToPosition(() -> 148, 3, m_wrist));
        NamedCommands.registerCommand("SetWristPosition SW Center",
                new MoveWristToPosition(() -> 125, 3, m_wrist, true));
        NamedCommands.registerCommand("SetWristPosition Four Piece Sides",
                new MoveWristToPosition(() -> 148.3, 3, m_wrist));
        NamedCommands.registerCommand("Shoot", new RunKickerWheel(3000.0, m_kicker).withTimeout(0.5));
        NamedCommands.registerCommand("Smart Shoot",
                new ShootUsingInterpolation(drivetrain, m_wrist, m_shooter, m_kicker));
        NamedCommands.registerCommand("Smart Shoot With Turning",
                new ShootUsingInterpolationWithCorrection(drivetrain, m_wrist, m_shooter, m_kicker));
        NamedCommands.registerCommand("Prepare Smart Shot", new SetShooterSpeed(() -> {
            return drivetrain.getShotInfo().getSpeed();
        }, 60.0, m_shooter));
        NamedCommands.registerCommand("Prepare Smart Shot Wrist", new MoveWristToPosition(() -> {
            return drivetrain.getShotInfo().getWrist();
        }, 3.0, m_wrist));
        NamedCommands.registerCommand("RunIntake 1",
                new RunIntakeSync(() -> 1.0, m_intake, m_kicker, false).withTimeout(3.0));
        NamedCommands.registerCommand("Retract", new Retract(m_wrist, m_arm));
        NamedCommands.registerCommand("RunIntake 0", new RunIntakeSync(() -> 0.0, m_intake, m_kicker, true));
        NamedCommands.registerCommand("RunIntake 1 Open Loop", new RunIntake(() -> 1.0, m_intake));
        NamedCommands.registerCommand("ExtendToAmp", new InstantCommand());
        NamedCommands.registerCommand("DropGamePiece", new InstantCommand());
        NamedCommands.registerCommand("RunKicker -0.5", new RunKickerWheel(-1500.0, m_kicker));
        NamedCommands.registerCommand("RunKicker 1", new RunKickerWheel(3000, m_kicker));
        drivetrain.configPathPlanner();
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.setDefaultOption("Shoot Preload (default)", new SequentialCommandGroup( // Sides
                new SetShooterSpeed(() -> 4000, 120, m_shooter),
                new MoveWristToPosition(() -> 125.0, 3.0, m_wrist, true),
                new RunKickerWheel(3000.0, m_kicker).withTimeout(0.5),
                new Retract(m_wrist, m_arm),
                new SetShooterSpeed(() -> 0, 120, m_shooter)));
        SmartDashboard.putData(m_autoChooser);
    }

    public Command getAutonomousCommand() {
        // return new InstantCommand();
        return m_autoChooser.getSelected();
    }

    public double getWristPosition() {
        return m_wrist.getWristPosition();
    }

    public double getArmPosition() {
        return m_arm.armPosition();
    }
}
