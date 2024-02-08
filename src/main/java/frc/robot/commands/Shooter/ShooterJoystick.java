// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterJoystick extends Command {
  /** Creates a new ShooterJoystick. */
  public ShooterJoystick(ShooterSubsystem mShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterOUTSpeed = RobotContainer.driver.getRawAxis(3);
    ShooterSubsystem.runShooter(shooterOUTSpeed);
    SmartDashboard.putNumber("Shooter Speed", shooterOUTSpeed);
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
