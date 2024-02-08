// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIntakeTilSight extends Command {
  /** Creates a new ShooterIntakeTilSight. */
  private ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  public ShooterIntakeTilSight() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.runShooter(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ShooterSubsystem.shooterNoteDetector.getValue() > 800){
      ShooterSubsystem.shooterMotor1.set(0);
      ShooterSubsystem.shooterMotor2.set(0);
      return true;
    } else {
      return false;
    }
    
  }
}
