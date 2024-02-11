// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public static final TalonFX IntakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
  public static final AnalogInput IntakeNoteDetector = new AnalogInput(Constants.IntakeConstants.IntakeNoteDetectorChannel);
  public static final TalonFX PositionIntakeMotor = new TalonFX(Constants.IntakeConstants.PositionIntakeMotorID);
  private static PositionDutyCycle IntakeUpDown = new PositionDutyCycle(0);
  private static TalonFXConfiguration IntakePositionconfig = new TalonFXConfiguration();
  public IntakeSubsystem() {
    resetIntake();
  }

  public static void LowerIntake(){
    PositionIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorDownkP;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.IntakeConstants.PositionIntakeMotorClosedLoopRamp;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(13.7));//Change this value when the motor is on the thing
  }

  public static void RiseIntake(){
    PositionIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.IntakeConstants.PositionIntakeMotorClosedLoopRamp;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(1.55));//Change this value as well
  }

  public static void setIntakePostition(double position){
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.IntakeConstants.PositionIntakeMotorClosedLoopRamp;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(position));
  }

  public static void resetIntake(){
    PositionIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
    PositionIntakeMotor.setPosition(0);
  }

  public static void ForwardIntake(){
    IntakeMotor.set(0.6);//0.5 if the bar is back where it was at the third closest to the back of the intake
  }

  public static void BackwardIntake(){
    IntakeMotor.set(-0.6);
  }

  public static void storeIntake(){
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake note detector", IntakeNoteDetector.getValue());
    SmartDashboard.putNumber("Intake Position Motor", PositionIntakeMotor.getPosition().getValueAsDouble());
  }
}