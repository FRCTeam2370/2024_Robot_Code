// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;

public class FindDistance extends Command {
  /** Creates a new FindDistance. */
  public FindDistance(Limelight myLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight", 1);
      }
      if (alliance.get() == Alliance.Blue){
        LimelightHelpers.setPipelineIndex("limelight", 0);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("FindDistance Command Executed");
    boolean tv = LimelightHelpers.getTV("");
    SmartDashboard.putBoolean("Apriltag Visible?", tv);
    double tx = LimelightHelpers.getTX("");
    SmartDashboard.putNumber("X Offset", tx);
    if (tv){
      double ty = LimelightHelpers.getTY("");
      double limelightMountHeight = Constants.LimelightConstants.limelightMoundHeight;
      double limelightMountPitch = Constants.LimelightConstants.limelightMountPitch;

      double angleToGoalDegrees = limelightMountPitch + ty;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      double distanceFromTarget = (57.13 - limelightMountHeight) / Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("Li DistanceFromTarget", distanceFromTarget);
      System.out.println("Sucess! Apriltag Visible");
    } else {
      System.out.println("No Apriltag Visible");
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
