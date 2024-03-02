// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class IRLEDs extends Command {
  private LEDSubsystem mLedSubsystem = new LEDSubsystem();
  private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  /** Creates a new IRLEDs. */
  public IRLEDs(LEDSubsystem mLedSubsystem, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mIntakeSubsystem = mIntakeSubsystem;
    this.mLedSubsystem = mLedSubsystem;
    addRequirements(mLedSubsystem, mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(IntakeSubsystem.IntakeNoteDetector.getValue() > 1400){
      LEDSubsystem.turnOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
