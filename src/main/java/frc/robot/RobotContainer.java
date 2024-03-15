// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeSync;
import frc.robot.commands.RunKickerWheel;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    private static final double MaxSpeed = Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps;
    private static final double MaxAngularRate = Math.PI;

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);
    // private final SetCurrentRequest current = new SetCurrentRequest();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final WristSubsystem m_wrist = new WristSubsystem(m_arm::armPosition);
    private final KickerSubsystem m_kicker = new KickerSubsystem();
    private final LEDSubsystem m_leds = new LEDSubsystem();

    private final SendableChooser<Command> m_autoChooser;

    private void configureBindings() {
        m_kicker.setDefaultCommand(new RunIntakeSync(() -> {
            return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
        }, m_intake, m_kicker, m_leds));
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        // drivetrain.setDefaultCommand(
        // drivetrain.applyRequest(() -> current));
        joystick.a().onTrue(new SetShooterSpeed(5200, 120, m_shooter)).onFalse(new SetShooterSpeed(0, 120, m_shooter));
        joystick.b().onTrue(new ParallelCommandGroup(
                new SetShooterSpeed(0, 120, m_shooter),
                new SequentialCommandGroup(
                        new MoveWristToPosition(90.0, 5.0, m_wrist),
                        new MoveArmToPosition(0, 10.0, m_arm),
                        new InstantCommand(() -> {
                            m_arm.setNeutral();
                        }, m_arm))));
        joystick.y().onTrue(new InstantCommand(drivetrain::seedFieldRelative, drivetrain));
        joystick.x().onTrue(new RunIntakeSync(() -> {
            return 1;
        }, m_intake, m_kicker, m_leds).withTimeout(4));
        joystick.leftBumper().onTrue(new ParallelCommandGroup(new MoveArmToPosition(90.0, 7.5, m_arm),
                new MoveWristToPosition(40.0, 7.5, m_wrist)));
        joystick.rightBumper().onTrue(new SequentialCommandGroup(
                new MoveArmToPosition(0.0, 7.5, m_arm),
                new InstantCommand(() -> {
                    m_arm.setNeutral();
                }, m_arm),
                new MoveWristToPosition(Preferences.getDouble("Subwoofer Wrist", 110.0), 10, m_wrist),
                new SetShooterSpeed(3500, 120, m_shooter)))
                .onFalse(new RunKickerWheel(3000.0, m_kicker).withTimeout(1.0)
                        .andThen(new SetShooterSpeed(0, 120, m_shooter)).andThen(new Retract(m_wrist, m_arm)));
        joystick.povDown().onTrue(new SequentialCommandGroup(
                new MoveArmToPosition(0.0, 7.5, m_arm),
                new InstantCommand(() -> {
                    m_arm.setNeutral();
                }, m_arm),
                new MoveWristToPosition(Preferences.getDouble("Podium Wrist", 110.0), 10, m_wrist),
                new SetShooterSpeed(5000, 120, m_shooter)))
                .onFalse(new RunKickerWheel(3000.0, m_kicker).withTimeout(1.0)
                        .andThen(new SetShooterSpeed(0, 120, m_shooter)).andThen(new Retract(m_wrist, m_arm)));
    }

    public RobotContainer() {
        SmartDashboard.putNumber("Set Current Request", 0.0);
        Preferences.initDouble("Subwoofer Wrist", 110.0);
        Preferences.initDouble("Podium Wrist", 110.0);
        configureBindings();
        // Register all of the commands for autos, then set up auto builder and the auto
        // chooser.
        NamedCommands.registerCommand("SetShooterSpeed 5000", new SetShooterSpeed(5000, 120, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 4000", new SetShooterSpeed(4000, 120, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 1000", new SetShooterSpeed(1000, 120, m_shooter));
        NamedCommands.registerCommand("SetShooterSpeed 0", new SetShooterSpeed(0, 120, m_shooter));
        NamedCommands.registerCommand("SetArmPosition 36", new MoveArmToPosition(36, 3, m_arm));
        NamedCommands.registerCommand("SetWristPosition 45",
                new MoveWristToPosition(Preferences.getDouble("Subwoofer Wrist", 110.0), 5.0, m_wrist));
        NamedCommands.registerCommand("SetWristPosition 81", new MoveWristToPosition(148.5, 3, m_wrist));
        NamedCommands.registerCommand("SetWristPosition 76", new MoveWristToPosition(143, 3, m_wrist));
        NamedCommands.registerCommand("SetWristPosition Four Piece Sides", new MoveWristToPosition(143.3, 3, m_wrist));
        NamedCommands.registerCommand("Shoot", new RunKickerWheel(3000.0, m_kicker).withTimeout(0.5));
        NamedCommands.registerCommand("RunIntake 1",
                new RunIntakeSync(() -> 1.0, m_intake, m_kicker, m_leds).withTimeout(3.0));
        NamedCommands.registerCommand("Retract", new Retract(m_wrist, m_arm));
        NamedCommands.registerCommand("RunIntake 0", new RunIntakeSync(() -> 0.0, m_intake, m_kicker, m_leds, true));
        NamedCommands.registerCommand("ExtendToAmp", new InstantCommand());
        NamedCommands.registerCommand("DropGamePiece", new InstantCommand());
        NamedCommands.registerCommand("RunKicker -0.5", new RunKickerWheel(-1500.0, m_kicker));
        drivetrain.configPathPlanner();
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_autoChooser);
    }

    public Command getAutonomousCommand() {
        // return new InstantCommand();
        return m_autoChooser.getSelected();
    }
}
