// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
  /** Creates a new AimShooter. */
  private ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  public AimShooter(ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mShooterSubsystem = mShooterSubsystem;
    addRequirements(mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.A.getAsBoolean() == true){
      ShooterSubsystem.aimShooter(28.57);
    } else if(RobotContainer.B.getAsBoolean() == true){
      ShooterSubsystem.aimShooter(0.1);
    } else if(RobotContainer.leftButton.getAsBoolean() == true){
      ShooterSubsystem.aimShooter(10.64);
    } else if(RobotContainer.righButton.getAsBoolean() == true){
      ShooterSubsystem.aimShooter(21.71);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.shooterAimMotor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
