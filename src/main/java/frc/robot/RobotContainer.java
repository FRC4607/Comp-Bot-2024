// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BumpWrist;
import frc.robot.commands.RunKickerWheel;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private static final double MaxSpeed = edu.wpi.first.math.util.Units.feetToMeters(17.3) - 0.5;
    private static final double MaxAngularRate = Math.PI;

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private final WristSubsystem m_wrist = new WristSubsystem();

    private final KickerSubsystem m_kicker = new KickerSubsystem();

    private final SendableChooser<Command> m_autoChooser;


    private void configureBindings() {
        m_intake.setDefaultCommand(new RunIntake(() -> {
            return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
        }, m_intake));
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        joystick.a().onTrue(new SetShooterSpeed(5200, m_shooter)).onFalse(new SetShooterSpeed(0, m_shooter));
        joystick.b().whileTrue(new RunKickerWheel(-1.0, m_kicker));
        joystick.y().onTrue(new InstantCommand(drivetrain::seedFieldRelative, drivetrain));
        joystick.povUp().onTrue(new BumpWrist(1.0, m_wrist));
        joystick.povDown().onTrue(new BumpWrist(-1.0, m_wrist));
        joystick.povLeft().onTrue(new BumpWrist(-0.1, m_wrist));
        joystick.povRight().onTrue(new BumpWrist(0.1, m_wrist));
    }

    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("SetShooterSpeed 5000", new InstantCommand());
        NamedCommands.registerCommand("SetWristPosition 45", new InstantCommand());
        NamedCommands.registerCommand("Shoot", new InstantCommand());
        NamedCommands.registerCommand("RunIntake 20", new RunIntake(()->-1.0, m_intake));
        NamedCommands.registerCommand("Retract", new InstantCommand());
        NamedCommands.registerCommand("RunIntake 0", new InstantCommand());
        NamedCommands.registerCommand("ExtendToAmp", new InstantCommand());
        NamedCommands.registerCommand("DropGamePiece", new InstantCommand());
        drivetrain.configPathPlanner();
        m_autoChooser =AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_autoChooser);
    }

    public Command getAutonomousCommand() {
        // return new InstantCommand();
        return m_autoChooser.getSelected();
    }
}
