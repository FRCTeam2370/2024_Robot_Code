// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightLEDs extends Command {
  private static LEDSubsystem mLedSubsystem = new LEDSubsystem();
  private static Limelight mLimelight = new Limelight();
  private static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  /** Creates a new LimelightLEDs. */
  public LimelightLEDs() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLedSubsystem, mLimelight, mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Limelight.InLongRange() == true){
      LEDSubsystem.setGreen();
    }else if(Limelight.InCloseRange() == true){
      LEDSubsystem.setGreen();
    }else{
      LEDSubsystem.setRed();
    }

    ShooterSubsystem.runShooter(-90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDSubsystem.turnOff();
    ShooterSubsystem.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
