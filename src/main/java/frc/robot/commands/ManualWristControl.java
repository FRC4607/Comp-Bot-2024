// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class ManualWristControl extends Command {
  private final DoubleSupplier m_upTrigger;
  private final DoubleSupplier m_downTrigger;

  private final WristSubsystem m_subsystem;

  private Measure<Angle> targetAngle;

  private static final double SCALE = 1.0 / 2.0;
  /** Creates a new ManualWristControl. */
  public ManualWristControl(DoubleSupplier up, DoubleSupplier down, WristSubsystem subsystem) {
    m_upTrigger = up;
    m_downTrigger = down;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = m_subsystem.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double addRotations = SCALE * (m_upTrigger.getAsDouble() - m_downTrigger.getAsDouble()) * 0.02;
    targetAngle = targetAngle.plus(Units.Rotations.of(addRotations));
    m_subsystem.setAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
