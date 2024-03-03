// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterToggle extends Command {

  private ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  /** Creates a new ShootForward. */
  public ShooterToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(RobotContainer.trigger(RobotContainer.driver, 3).getAsBoolean() == true){
    ShooterSubsystem.runShooter(-90);//0.8
  } else if(RobotContainer.trigger(RobotContainer.driver, 2).getAsBoolean() == true){
    ShooterSubsystem.runShooter(-10);
  } else if(RobotContainer.driver.getPOV() == 270){
    ShooterSubsystem.runShooter(10);
  } else {
    ShooterSubsystem.runShooter(0);
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
