package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootOverDefense extends SequentialCommandGroup {
    public ShootOverDefense(ArmSubsystem arm, WristSubsystem wrist, KickerSubsystem kicker, ShooterSubsystem shoot) {
        super(
            new ParallelCommandGroup(
                new MoveArmToPosition(90, 1, arm),
                new MoveWristToPosition(() -> 25.0, 1, wrist),
                new SetShooterSpeed(() -> 3000, 120, shoot)
            ).withTimeout(3),
            new RunKickerWheel(3000, kicker).withTimeout(0.5),
            new ParallelCommandGroup(
                new RunKickerWheel(0, kicker),
                new SetShooterSpeed(() -> 0, 120, shoot),
                new Retract(wrist, arm)
            )
        );
        addRequirements(arm, wrist, kicker, shoot);
    }
}

