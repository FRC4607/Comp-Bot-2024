// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command that allows setting the position of the wrist manually.
 */
public class ManualWristControl extends Command {
  private final DoubleSupplier m_value;

  private final WristSubsystem m_subsystem;

  private double targetAngle;

  private static final double SCALE = 1.0 / 2.0 * 360.0;
  /** Creates a new ManualWristControl. */
  public ManualWristControl(DoubleSupplier value, WristSubsystem subsystem) {
    m_value = value;
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
    double addRotations = SCALE * m_value.getAsDouble() * 0.02;
    targetAngle += addRotations;
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
