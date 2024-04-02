// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.commands.LED.LimelightLEDs;

import frc.robot.subsystems.Limelight;

public class AlignToTag extends Command {
private static double velocityx;
private static double velocityy;
private static CommandSwerveDrivetrain mDrivetrain = RobotContainer.drivetrain;
private static Limelight mLimelight;
  /** Creates a new AlignToTag. */
  public AlignToTag(Limelight mLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrivetrain, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("----------------Made It to Align To Tag----------------------");
    double rotationalRate = Limelight.txSlowly();
    SmartDashboard.putNumber("rotational rate in Align to Tag", rotationalRate);
    mDrivetrain.applyRequest(()-> RobotContainer.drive.withRotationalRate(RobotContainer.LimelightTurnPID.calculate(rotationalRate)));
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
