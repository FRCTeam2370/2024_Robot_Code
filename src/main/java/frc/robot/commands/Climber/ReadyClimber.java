// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ReadyClimber extends Command {
  private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  /** Creates a new ReadyClimber. */
  public ReadyClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimberSubsystem, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.storeIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ClimberSubsystem.LowerLimitSwitch.get() == false){
      ClimberSubsystem.Climb(ClimberSubsystem.ClimberMotor.getPosition().getValueAsDouble());
    }else{
      ClimberSubsystem.Climb(30);
    }
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
