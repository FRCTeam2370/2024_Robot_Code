// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;

public class Amped extends Command {
  private LEDSubsystem mLedSubsystem = new LEDSubsystem();
  /** Creates a new Amped. */
  public Amped(LEDSubsystem mLedSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mLedSubsystem = mLedSubsystem;
    addRequirements(mLedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(LEDSubsystem.isBlue() == true){
      LEDSubsystem.setBlue();
      }else if(LEDSubsystem.isBlue() == false){
      LEDSubsystem.setPink();
      }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDSubsystem.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
