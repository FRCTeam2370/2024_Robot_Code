// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.intakeToggle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {
  /** Creates a new RunShooter. */
  ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public RunShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.runShooter(-0.6);
    IntakeSubsystem.BackwardIntake();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
