package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SourcePass extends ParallelCommandGroup {
    public SourcePass(ArmSubsystem arm, WristSubsystem wrist, ShooterSubsystem shoot) {
        super(
                new MoveArmToPosition(18, 2, arm),
                new MoveWristToPosition(() -> 180, 2, wrist),
                new SetShooterSpeed(() -> 4500.0, 120, shoot),
                new RunCommand(() -> {
                }));
    }
}
