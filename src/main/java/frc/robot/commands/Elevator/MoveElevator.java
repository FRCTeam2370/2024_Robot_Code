// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveElevator extends Command {
  /** Creates a new MoveElevator. */
  private ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private double position;
  private double position1;
  public MoveElevator(double position, double position1) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mElevatorSubsystem);
    this.position = position;
    this.position1 = position1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.setElevator(position);
    ShooterSubsystem.aimShooter(position1);
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
    /*if(ElevatorSubsystem.elevatorMotor.getPosition().getValueAsDouble() >= position - 0.075 || ElevatorSubsystem.elevatorMotor.getPosition().getValueAsDouble() <= position + 0.075){
      return true;
    }*/
    return true;
  }
}
