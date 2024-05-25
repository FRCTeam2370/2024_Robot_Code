// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Shooter.ShootClose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightLEDs extends Command {
  private static LEDSubsystem mLedSubsystem = new LEDSubsystem();
  private static Limelight mLimelight = new Limelight();
  private static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  /** Creates a new LimelightLEDs. */
  public LimelightLEDs() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLedSubsystem, mLimelight, mShooterSubsystem, mIntakeSubsystem, mElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Limelight.InLongRange() == true){
      LEDSubsystem.setGreen();
      ElevatorSubsystem.setElevator(0.1);
      IntakeSubsystem.setIntakePostition(0.01154);// 0.009
      ShooterSubsystem.aimShooter(5.425);//5.35, //5.43, //4.75 for some ODD reason
    }else if(Limelight.InCloseRange() == true){
      LEDSubsystem.setGreen();
      ElevatorSubsystem.setElevator(0.1);
      IntakeSubsystem.setIntakePostition(0.00154);
      ShooterSubsystem.aimShooter(3.6);
    }/*else if(Limelight.InMidRange() == true){
      LEDSubsystem.setGreen(); 
      ElevatorSubsystem.setElevator(0.1);
      IntakeSubsystem.setIntakePostition(0.013);
      ShooterSubsystem.aimShooter(5.6);
    }*/else{
      LEDSubsystem.setRed();
    }
    

    ShooterSubsystem.runShooter(-95);
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
