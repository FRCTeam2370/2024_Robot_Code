// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /*  Creates a new ShooterSubsystem. */
  public static CANcoder shooterAimCANCoder = new CANcoder(Constants.ShooterConstants.ShooterAimCANcoderID);
  public static TalonFX shooterMotor1 = new TalonFX(Constants.ShooterConstants.ShooterMotor1ID);
  public static TalonFX shooterMotor2 = new TalonFX(Constants.ShooterConstants.ShooterMotor2ID);
  public static CANSparkMax shooterAimMotor = new CANSparkMax(1, MotorType.kBrushless);
  private static TalonFXConfiguration shooterMotor1config = new TalonFXConfiguration();
  private static TalonFXConfiguration shooterMotor2config = new TalonFXConfiguration();
  private static TalonFXConfiguration AimMotorConfig = new TalonFXConfiguration();
  private static PositionDutyCycle mPositionDutyCycle = new PositionDutyCycle(0);
  public static AnalogInput shooterNoteDetector = new AnalogInput(Constants.ShooterConstants.ShooterNoteDetectorChannel);
  private static SparkPIDController aimMotorPID = shooterAimMotor.getPIDController();
  private static VelocityDutyCycle shooterVelocityDC = new VelocityDutyCycle(0);
  public ShooterSubsystem(){
    resetAimShooterMotor();
    resetShooter();
  }

  public static void resetShooter(){
    shooterMotor1config.Slot0.kP = 0.03;
    shooterMotor2config.Slot0.kP = 0.03;
    shooterMotor1config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.ShooterConstants.ShooterClosedLoopRamp;
    shooterMotor2config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.ShooterConstants.ShooterClosedLoopRamp;
    shooterMotor1.getConfigurator().apply(shooterMotor1config);
    shooterMotor2.getConfigurator().apply(shooterMotor2config);
    shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
  }

  public static void runShooter(double shooterSpeed){
      /*shooterMotor1.set(shooterSpeed*0.95);
      shooterMotor2.set(shooterSpeed);//* 1.05*/
      shooterMotor1.setControl(shooterVelocityDC.withVelocity(shooterSpeed*0.95));
      shooterMotor2.setControl(shooterVelocityDC.withVelocity(shooterSpeed)); 
  }

  public static void aimShooter(double position){
    aimMotorPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public static void debugAngle(double addToPosition){
    double position = 0;
    position += addToPosition / 10;
    aimMotorPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  
  public static void resetAimShooterMotor(){
    /*AimMotorConfig.Slot0.kP = Constants.ShooterConstants.ShooterAimMotorkP;
    AimMotorConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
    AimMotorConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
    shooterAimMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterAimMotor.getConfigurator().apply(AimMotorConfig);
    shooterAimMotor.setPosition(0);*/
    aimMotorPID.setP(0.05);
    aimMotorPID.setFF(0.007);
    shooterAimMotor.setClosedLoopRampRate(0);
    shooterAimMotor.getEncoder().setPosition(0);
  }

  public static void intakeTilSight(){
  if(shooterNoteDetector.getValue() > 850){
    shooterMotor1.set(0);
    shooterMotor2.set(0);
  } else {
    shooterMotor1.set(-0.1);
    shooterMotor2.set(0.1);
  }
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Aim Shooter CANcoder Value", shooterAimCANCoder.getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Aim Shooter motor value", shooterAimMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter Note Detector", shooterNoteDetector.getValue());
    SmartDashboard.putNumber("something", shooterAimMotor.getEncoder().getPosition());
  }
}
