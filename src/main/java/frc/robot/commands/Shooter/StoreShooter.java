// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StoreShooter extends Command {
  /** Creates a new AimShooterDown. */
  ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public StoreShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem, mElevatorSubsystem, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.setElevator(0.1);
    IntakeSubsystem.setIntakePostition(0.026);
    ShooterSubsystem.aimShooter(5.2);
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
