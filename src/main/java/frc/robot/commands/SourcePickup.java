package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SourcePickup extends ParallelCommandGroup {
    public SourcePickup(ArmSubsystem arm, WristSubsystem wrist) {
        super(
            new MoveArmToPosition(90, 1, arm),
            new MoveWristToPosition(() -> -15.0, 1, wrist)
        );
    }
}
