// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class intakeToggle extends Command {
  /** Creates a new intakeToggle. */
  private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private LEDSubsystem mLEDSubsystem = new LEDSubsystem();
  public intakeToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem, mLEDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.LowerIntake();
    LEDSubsystem.turnOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.ForwardIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.IntakeMotor.set(0);
    IntakeSubsystem.RiseIntake();
    LEDSubsystem.youGotTheThing();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if(IntakeSubsystem.IntakeNoteDetector.getValue() < 975){//1350 for if the bar on the intake is third from the back
    return true;
   }else{
    return false;
   }
     
    
    
  }
}
