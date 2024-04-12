package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SourcePassOver extends ParallelCommandGroup {
    public SourcePassOver(ArmSubsystem arm, WristSubsystem wrist, ShooterSubsystem shoot) {
        super(
                new MoveArmToPosition(() -> 0, 2, arm),
                new MoveWristToPosition(() -> 140, 2, wrist),
                new SetShooterSpeed(() -> 3200.0, 120, shoot),
                new RunCommand(() -> {
                }));
    }
}
