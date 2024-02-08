// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public static TalonFX ClimberMotor = new TalonFX(8);
  public static CANcoder ClimberCANcoder = new CANcoder(10);
  private static TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();
  private static PositionDutyCycle climberDutyCycle = new PositionDutyCycle(0);

  public ClimberSubsystem() {
    resetClimber();
  }

  public static void readyClimber(double position){
    ClimberMotor.setControl(climberDutyCycle.withPosition(position));
  }

  public static void Climb(double position){
    ClimberMotor.setControl(climberDutyCycle.withPosition(position));
    
  }

  public static void resetClimber(){
    ClimberMotor.setPosition(0);
    ClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    ClimberConfig.Slot0.kP = Constants.ClimberConstants.ClimberkP;
    ClimberConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.ClimberConstants.ClimberClosedLoopRamp;
    ClimberMotor.getConfigurator().apply(ClimberConfig);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", ClimberMotor.getPosition().getValueAsDouble());
  }
}
