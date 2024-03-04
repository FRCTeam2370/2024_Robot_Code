// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntake extends Command {
  /** Creates a new MoveIntake. */
  private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public MoveIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.LowerIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.getRawButton(10) == true){
      IntakeSubsystem.IntakeMotor.set(0.1);
    } else if (RobotContainer.operator.getRawButton(9) == true){
      IntakeSubsystem.IntakeMotor.set(-0.1);
    }else{
      IntakeSubsystem.IntakeMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.IntakeMotor.set(0);
    IntakeSubsystem.RiseIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
