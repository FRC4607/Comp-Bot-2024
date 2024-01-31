// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ManagePower extends Command {

  ArmSubsystem m_armSubsystem;
  ClimberSubsystem m_climberSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  KickerSubsystem m_kickerSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  WristSubsystem m_wristSubsystem;

  private int m_value;

  /** Creates a new ManagePower. */
  public ManagePower() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_value = 1;

    if (m_climberSubsystem.isRunning) {
      m_climberSubsystem.setClimberPower(power());
      m_value ++;
    }

    if (m_shooterSubsystem.isRunning) {
      m_shooterSubsystem.setShooterPower(power());
      m_value ++;
    }

    if (m_armSubsystem.isRunning) {
      m_armSubsystem.setArmPower(power());
      m_value ++;
    }

    if (m_wristSubsystem.isRunning) {
      m_wristSubsystem.setWristPower(power());
      m_value ++;
    }

    if (m_kickerSubsystem.isRunning) {
      m_kickerSubsystem.setKickerPower(power());
      m_value ++;
    }

    if (m_intakeSubsystem.isRunning) {
      m_intakeSubsystem.setIntakePower(power());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Decides the new power coefficient based on subsystem priority.
   * 
   * @return the new power coefficient.
   */
  private double power() {

    // will always return 1.0, values can be modified later
    switch (m_value) {
      case 1:
        return 1.0;
      case 2:
        return 1.0;
      case 3:
        return 1.0;
      case 4:
        return 1.0;
      case 5:
        return 1.0;
      case 6:
        return 1.0;
      default:
        return 1.0;
    }

  }
}
