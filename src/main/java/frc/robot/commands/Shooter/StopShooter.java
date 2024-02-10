// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Intake.intakeToggle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command {
  /** Creates a new StopShooter. */
  ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public StopShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.runShooter(0);
    IntakeSubsystem.IntakeMotor.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
