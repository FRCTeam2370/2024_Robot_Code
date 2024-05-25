// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class ResetIntakeManual extends Command {
  private static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  /** Creates a new ResetIntakeManual. */
  public ResetIntakeManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.getRawAxis(1)> 0.9){
      IntakeSubsystem.moveIntakeBack(-0.15);
    }else if(RobotContainer.operatorM2.getAsBoolean() == true){
      IntakeSubsystem.resetIntakePose();
    }else{
      IntakeSubsystem.moveIntakeBack(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.moveIntakeBack(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
