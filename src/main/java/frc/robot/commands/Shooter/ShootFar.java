// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFar extends Command {
  /** Creates a new ShootFar. */
  ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public ShootFar() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mElevatorSubsystem, mIntakeSubsystem, mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.setElevator(0.1);
    IntakeSubsystem.setIntakePostition(1.56);
    ShooterSubsystem.aimShooter(6.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
